#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <unordered_map>
#include <string>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/gui/gui.hh>
#include <gazebo/rendering/rendering.hh>
#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>

// If automatic controlling mode is demanded
#define AUTOMATIC_MODE 1

#define TRAINED_MODEL 1

// Imu sensors limits
#define UPPER_BOUNDARY 2150
#define LOWER_BOUNDARY 1950

// Stress threshold
#define STRESS_THRESHOLD 0.5

#if AUTOMATIC_MODE // automatic mode
#define CLOSEST_DISTANCE 0.32
#define SIDE_DISTANCE 0.32
#define TURNING_DEGREE 5.0
#else // just obstacle avoidance
#define CLOSEST_DISTANCE 0.1
#define SIDE_DISTANCE 0.1
#define TURNING_DEGREE 30.0
#define LIMIT_SPEED_DIST 0.8
#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define LINEAR_VELOCITY 0.2
#define ANGULAR_VELOCITY 0.5

#define GAME_TIME 300 // sec
#define TURTLEBOT_GEAR_SPEED 2

enum states
{
    GET_TB3_DIRECTION = 0,
    TB3_DRIVE_FORWARD,
    TB3_RIGHT_TURN,
    TB3_LEFT_TURN,
    TB3_DRIVE_BACKWARD,
};

enum directions
{
    CENTER = 0,
    FLEFT,
    FRIGHT,
    BACK,
};

enum Events
{
    GAME_NOT_STARTED,
    COLLIDE_WITH_WALL,
    COLLIDE_WITH_BOX,
    ROBOT_CONTROL,
    SHARED_ROBOT_CONTROL,
};

Events Curret_event = ROBOT_CONTROL;

// Define the experiment type
static bool SHARED_CONTROL_MECHANISM{true};

// Variables
double escape_range_ = TURNING_DEGREE * DEG2RAD;
double check_forward_dist_ = CLOSEST_DISTANCE;
double check_side_dist_ = SIDE_DISTANCE;

double scan_data_[3] = {0.0, 0.0, 0.0};

double tb3_pose_{0.0};
double prev_tb3_pose_{0.0};

// Init variables
float imu_pitch{0.0f};
float imu_yaw{0.0f};
float speed(0.1); // Linear velocity (m/s)
float linear_gear{1};
float turn(1.0); // Angular velocity (rad/s)
float min_point{0.0};

// Score and time parameters
int score{100};

ros::Publisher node_publisher;
ros::Publisher node_score_pub;
// Create Twist message
geometry_msgs::Twist twist;

ros::ServiceClient set_model_state_client;

bool is_obstacle{false};
int obstacle_degree{-1};
float obstacle_distance{-1.0f};

bool is_stressful{false};
bool inform_control_given_to_user{false};

bool is_finishing_point{false}; // Robot will finish when it hits to finishin point

static std::string string_solver(std::string const &str)
{
    std::string temp_str;
    size_t count_underscore{0};
    for (auto &i : str)
    {
        if (i == ':')
            return temp_str;

        /* If our boxes name has an underscore (box_model_2)
            just ignore the second one */
        if (i == '_')
            count_underscore++;

        if (count_underscore > 1)
            return temp_str;
        temp_str += i;
    }
    return temp_str;
}

void set_model_state(std::string model_name, std::string reference_frame, geometry_msgs::Pose pose)
{
    // Set model service struct
    gazebo_msgs::SetModelState setmodelstate;

    // Model state msg
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    // modelstate.twist = model_twist;

    setmodelstate.request.model_state = modelstate;

    // Call the service
    bool success = set_model_state_client.call(setmodelstate);
}


bool box_collision_occured_flag{false};
bool wall_collision_occured_flag{false};

void gazebo_contact_read(const ConstContactsPtr &msg)
{
    static const std::string turtl_name = "turtlebot3_burger";
    static const std::string box_name = "box_model";
    // static const std::string wall_name = "labyrinth";
    static const std::string challenging_path_name = "challenging_path";
    static const std::string finish_point = "finish_sign";

    bool box_collision_occured{false};
    bool wall_collision_occured{false};

    for (int i = 0; i < msg->contact_size(); ++i)
    {
        auto col_1 = string_solver(msg->contact(i).collision1());
        auto col_2 = string_solver(msg->contact(i).collision2());

        if (col_1 == turtl_name)
        {
            /* if col1 is turtlebot instance, check whether col_2 
                   is equal the instances we are looking for */
            if (col_2 == challenging_path_name)
            {
                wall_collision_occured = true;
                wall_collision_occured_flag = true;
            }
            if ((col_2 == box_name))
            {
                box_collision_occured = true;
                box_collision_occured_flag = true;
            }

            if (col_2 == finish_point)
                is_finishing_point = true;
        }
        else if (col_2 == turtl_name)
        {
            /* if col2 is turtlebot instance, check whether col_1 
                   is equal the instances we are looking for */
            if (col_1 == challenging_path_name)
            {
                wall_collision_occured = true;
                wall_collision_occured_flag = true;
            }

            if ((col_1 == box_name))
            {
                box_collision_occured = true;
                box_collision_occured_flag = true;
            }

            if (col_1 == finish_point)
                is_finishing_point = true;
        }

        // if (SHARED_CONTROL_MECHANISM)
        // {
        //     if (is_stressful && box_collision_occured)
        //     {
        //         box_collision_occured = false;
        //     }
        // }

        if ((box_collision_occured || wall_collision_occured) && !inform_control_given_to_user)
        { // if the person gets stressed, help him by not returing starting point and not decreasing score
            std::cout << "ROBOT COLLIDED WITH INSTANCE !!!" << std::endl;
            // if the robot collide with obstacles, return to the initial point
            geometry_msgs::Pose model_pose;
            model_pose.position.x = -4.86;
            model_pose.position.y = -4.23;
            model_pose.position.z = 0;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 1.63;
            model_pose.orientation.w = 1.63;

            score -= 0.25; // Decrease the overall score depending on the collision level
            wall_collision_occured = false;
            box_collision_occured = false;

            set_model_state(turtl_name, "world", model_pose);
        }
    }
}

void stress_handler(const std_msgs::Float32MultiArray::ConstPtr &received_stress)
{
    /*  
    The GSR and PPG values will be processed in this function, and 
    it will be determined whether the person gets stressed or not
*/

#if TRAINED_MODEL

    is_stressful = static_cast<bool>(received_stress->data[0]);
    SHARED_CONTROL_MECHANISM = static_cast<bool>(received_stress->data[1]);

#else // reservior computing method
    std::cout << received_stress->data << "   ";
    if (received_stress->data >= STRESS_THRESHOLD)
        is_stressful = true;
    else
        is_stressful = false;
#endif
    std::cout << "Stress: ";
    std::cout << std::boolalpha << is_stressful << std::endl;
}


Events event_detector()
{
    if(wall_collision_occured_flag)
    {
        wall_collision_occured_flag = false;
        return Events::COLLIDE_WITH_WALL;
    }

    if(box_collision_occured_flag)
    {
        box_collision_occured_flag = false;
        return Events::COLLIDE_WITH_BOX;
    }

    if(is_stressful)
    {
        return Events::SHARED_ROBOT_CONTROL;
    } else {
        return Events::ROBOT_CONTROL;
    }
}


void odomHandler(const nav_msgs::Odometry::ConstPtr &msg)
{
    // orientation z is our rotation axis and x, y axes are the coordinates
    double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

    // This yaw angle (heading angle) produce only between -pi and pi.
    // https://answers.ros.org/question/12668/how-to-measure-an-angle/
    tb3_pose_ = atan2(siny, cosy);
}

void sensorHandler(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    // Assign the minimum value to the vector
    // auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    // obstacle_distance = *min_it;

    // // Check is there any possible obstacle
    // for (size_t degree = 0; degree < msg->ranges.size(); ++degree)
    // {
    //     if (msg->ranges[degree] < CLOSEST_DISTANCE)
    //     { // Obstacle detected
    //         is_obstacle = true;
    //         obstacle_degree = static_cast<int>(degree);
    //         // std::cout << msg->ranges[degree] << std::endl;
    //         break;
    //     }
    //     else
    //     { // No any obstacle detected
    //         is_obstacle = false;
    //         obstacle_degree = -1;
    //     }
    // }

    const size_t ds_cnt = 30;
    const size_t ds_lr = 30;             // (15 to 45) - so 30 degree for left and right sides
    float scan_range_of_angles[4]{0.0f}; // 15-345, 15-45, 315-345
    scan_range_of_angles[0] = std::min(msg->ranges[std::distance(msg->ranges.begin(), std::min_element(msg->ranges.begin(), msg->ranges.begin() + ds_cnt / 2))],
                                       msg->ranges[std::distance(msg->ranges.begin(), std::min_element(msg->ranges.end() - ds_cnt / 2, msg->ranges.end()))]);   // Center
    scan_range_of_angles[1] = msg->ranges[std::distance(msg->ranges.begin(), std::min_element(msg->ranges.begin() + ds_cnt / 2, msg->ranges.begin() + ds_lr))]; // Fleft
    scan_range_of_angles[2] = msg->ranges[std::distance(msg->ranges.begin(), std::min_element(msg->ranges.end() - ds_lr, msg->ranges.end() - ds_cnt / 2))];     // Fright
    scan_range_of_angles[3] = msg->ranges[std::distance(msg->ranges.begin(), std::min_element(msg->ranges.begin() + 90, msg->ranges.begin() + 150))];           // backward

    for (int num = 0; num < 3; num++)
    {
        if (std::isinf(scan_range_of_angles[num]))
        {
            scan_data_[num] = msg->range_max;
        }
        else
        {
            scan_data_[num] = scan_range_of_angles[num];
        }
    }

    // if only one sensor value is enough for the obstacle avoidance algorithm, we can use following code
    // const uint16_t scan_angle[3] = {0, 30, 330};

    // for (int num = 0; num < 3; num++)
    // {
    //     if (std::isinf(msg->ranges.at(scan_angle[num])))
    //     {
    //         scan_data_[num] = msg->range_max;
    //     }
    //     else
    //     {
    //         scan_data_[num] = msg->ranges.at(scan_angle[num]);
    //     }
    // }
}

void user_imu_control(float pitch, float yaw, float current_speed)
{
    if (pitch > UPPER_BOUNDARY)
    {
        twist.linear.x = current_speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
    }
    else if (pitch < LOWER_BOUNDARY)
    {
        twist.linear.x = -current_speed;
        twist.linear.y = 0;
        twist.linear.z = 0;
    }
    else
    {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
    }

    // Angular acceleration - roll movement of shimmer sensor
    if (yaw > UPPER_BOUNDARY)
    {
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = -turn;
    }
    else if (yaw < LOWER_BOUNDARY)
    {
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = turn;
    }
    else
    {
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
}

void updatecommandVelocity(double linear, double angular)
{
    twist.linear.x = linear;
    twist.angular.z = angular;
}

template <typename S>
void update_linear_gear(S sensor)
{

    if (sensor > UPPER_BOUNDARY && sensor < UPPER_BOUNDARY + 100)
    {
        linear_gear = TURTLEBOT_GEAR_SPEED;
    }
    else if (sensor > UPPER_BOUNDARY + 100 && sensor < UPPER_BOUNDARY + 200)
    {
        linear_gear = 2 * TURTLEBOT_GEAR_SPEED;
    }
    else if (sensor > UPPER_BOUNDARY + 200 && sensor < UPPER_BOUNDARY + 300)
    {
        linear_gear = 3 * TURTLEBOT_GEAR_SPEED;
    }
    else if (sensor > UPPER_BOUNDARY + 300 && sensor < UPPER_BOUNDARY + 400)
    {
        linear_gear = 4 * TURTLEBOT_GEAR_SPEED;
    }
    else if (sensor > UPPER_BOUNDARY + 400 && sensor < UPPER_BOUNDARY + 500)
    {
        linear_gear = 5 * TURTLEBOT_GEAR_SPEED;
    }
    else
    {
        linear_gear = 5 * TURTLEBOT_GEAR_SPEED;
    }
}

bool obstacle_avoidance()
{
    static uint8_t turtlebot3_state = 0;

    // limit the speed during obstacle avoidance
    auto limit_the_speed = [](auto sp)
    {
        if (sp > LINEAR_VELOCITY)
            sp = LINEAR_VELOCITY;
        return sp;
    };

    switch (turtlebot3_state)
    {

#if AUTOMATIC_MODE
    case GET_TB3_DIRECTION:

        if (scan_data_[CENTER] > check_forward_dist_)
        {
            if (scan_data_[FLEFT] < check_side_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_RIGHT_TURN;
            }
            else if (scan_data_[FRIGHT] < check_side_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_LEFT_TURN;
            }
            else
            {
                // turtlebot3_state = TB3_DRIVE_FORWARD;
                // forward and bacward will be given to the user
                auto current_speed = limit_the_speed(speed * linear_gear);
                user_imu_control(imu_pitch, imu_yaw, current_speed);
            }
        }

        if (scan_data_[CENTER] < check_forward_dist_)
        {
            // prev_tb3_pose_ = tb3_pose_;
            // turtlebot3_state = TB3_RIGHT_TURN;

            if (scan_data_[FLEFT] < check_side_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_RIGHT_TURN;
            }
            else if (scan_data_[FRIGHT] < check_side_dist_)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_LEFT_TURN;
            }
        }
        break;
#else
    case GET_TB3_DIRECTION:

        // for (int i = 0; i < 4; i++)
        // {
        //     std::cout << scan_data_[i] << " ";
        // }
        // std::cout << obstacle_degree << std::endl;

        if (scan_data_[CENTER] < check_forward_dist_ || scan_data_[FLEFT] < check_forward_dist_ ||
            scan_data_[FRIGHT] < check_forward_dist_)
        { // if the robot is very close to the obstacle
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state = TB3_DRIVE_BACKWARD;
        }
        else if (scan_data_[BACK] < check_forward_dist_)
        {
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state = TB3_DRIVE_FORWARD;
        }
        else //if (scan_data_[CENTER] > check_forward_dist_)
        {
            if (scan_data_[FLEFT] < 0.5)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_RIGHT_TURN;
            }
            else if (scan_data_[FRIGHT] < 0.5)
            {
                prev_tb3_pose_ = tb3_pose_;
                turtlebot3_state = TB3_LEFT_TURN;
            }
            else
            {
                turtlebot3_state = TB3_DRIVE_FORWARD;
            }
        }
        break;

#endif
    case TB3_DRIVE_BACKWARD:
        updatecommandVelocity(-LINEAR_VELOCITY, 0.0);
        turtlebot3_state = GET_TB3_DIRECTION;
        break;

    case TB3_DRIVE_FORWARD:
        updatecommandVelocity(LINEAR_VELOCITY, 0.0);
        turtlebot3_state = GET_TB3_DIRECTION;
        break;

    case TB3_RIGHT_TURN:
        if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
            turtlebot3_state = GET_TB3_DIRECTION;
        else
            updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
        break;

    case TB3_LEFT_TURN:
        if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
            turtlebot3_state = GET_TB3_DIRECTION;
        else
            updatecommandVelocity(0.0, ANGULAR_VELOCITY);
        break;

    default:
        turtlebot3_state = GET_TB3_DIRECTION;
        break;
    }

    return true;
}

/*  This function is used for control the robot with using IMU sensor values
 *  Besides, the person's physiological parameters will be captured to figure out 
 *  the stressful situations and in that circumstances, if the robot in danger, 
 *  we can intervene to the robot to avoid unintended consequences like accidents.
 */
void control_robot(const std_msgs::Float32MultiArray::ConstPtr &array)
{

    // update the gear rate to make the robot faster or slower

    imu_pitch = array->data[0];
    imu_yaw = array->data[1];

    update_linear_gear(imu_pitch);

    std_msgs::Float32MultiArray score_and_time;
    float elapsed_time{0.0f};
    Events get_event = event_detector();

    if (!is_finishing_point)
    {
        static bool is_started = true;
        static float time_stamp = 0.0;
        if (is_started)
        {
            time_stamp = ros::Time::now().toSec();
            is_started = false;
        }

        elapsed_time = ros::Time::now().toSec() - time_stamp;

        if (elapsed_time > GAME_TIME || (score <= 0))
        {
            // The end of the game - time is up or score is zero
            geometry_msgs::Pose model_pose;
            model_pose.position.x = -9.5;
            model_pose.position.y = -12.5;
            model_pose.position.z = 0;
            model_pose.orientation.x = 0.0;
            model_pose.orientation.y = 0.0;
            model_pose.orientation.z = 0.0;
            model_pose.orientation.w = 0.0;
            // std::cout << "END GAME..." << std::endl;
            set_model_state("turtlebot3_burger", "world", model_pose);

            static bool send_one_time{true};
            if (send_one_time)
            {
                send_one_time = false;
                if (score < 0)
                    score_and_time.data.push_back(0);
                else
                    score_and_time.data.push_back(score);
                score_and_time.data.push_back(GAME_TIME - elapsed_time);
                score_and_time.data.push_back(0); // Game has not finished succesfully
                score_and_time.data.push_back(static_cast<float>(inform_control_given_to_user));
                score_and_time.data.push_back(static_cast<float>(get_event)); 

                node_score_pub.publish(score_and_time);
            }
        }
        else
        {
            score_and_time.data.push_back(score);
            score_and_time.data.push_back(GAME_TIME - elapsed_time);
            score_and_time.data.push_back(0); // Game has not been finished succesfully
            score_and_time.data.push_back(static_cast<float>(inform_control_given_to_user));
            score_and_time.data.push_back(static_cast<float>(get_event)); 
            node_score_pub.publish(score_and_time);
        }
    }
    else if (is_finishing_point) // if the person finishes the path succesfully
    {
        static bool inform_success_one_time{true};
        if (inform_success_one_time)
        {
            inform_success_one_time = false;
            score_and_time.data.push_back(score);
            score_and_time.data.push_back(GAME_TIME - ros::Time::now().toSec());
            score_and_time.data.push_back(1); // The Game has been finished succesfully
            score_and_time.data.push_back(static_cast<float>(inform_control_given_to_user));
            score_and_time.data.push_back(static_cast<float>(get_event)); 

            node_score_pub.publish(score_and_time);

            std::cout << "Congratulations !!..." << std::endl;
        }
    }

    if (SHARED_CONTROL_MECHANISM)
    { // if the experiment type is with shared control
        // is_stressful = true; // for testing purpose
        static bool count_time_after_stress{false};
        static bool check_initial_time{false};
        static float count_down_initial_time{0.0f};
        // if the user gets stressed, check around to see whether there is an obstacle
        if (is_stressful && !is_finishing_point)
        {
            obstacle_avoidance();
            count_time_after_stress = true;
            check_initial_time = true;
            inform_control_given_to_user = false;
        }
        else if (count_time_after_stress && !is_finishing_point)
        {
            // Continue to control the robot for 3 sec after stressful condition
            // and let the user know the control will be given again.
            if (check_initial_time)
            {
                count_down_initial_time = ros::Time::now().toSec();
                check_initial_time = false;
            }

            if ((ros::Time::now().toSec() - count_down_initial_time) < 5)
            {
                // continue to obstacle avoidance for 5 sec
                inform_control_given_to_user = true;
                obstacle_avoidance();
            }
            else
            {
                inform_control_given_to_user = true;
                // Stop the robot and give to the user
                updatecommandVelocity(0, 0);
                count_time_after_stress = false;
            }
        }
        else if (!is_finishing_point)
        {
            inform_control_given_to_user = false;
            // Use the upcomming shimmer3 imu values to control the robot
            // Linear acceleration - pitch movement of shimmer sensor
            user_imu_control(imu_pitch, imu_yaw, speed * linear_gear);
        }
        else if (is_finishing_point)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
        }
    }
    else
    {
        // if the experiment type is without shared control
        if (!is_finishing_point)
        {
            inform_control_given_to_user = false;
            // Use the upcomming shimmer3 imu values to control the robot
            // Linear acceleration - pitch movement of shimmer sensor
            user_imu_control(imu_pitch, imu_yaw, speed * linear_gear);
        }
        else if (is_finishing_point)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;
        }
    }

    // Publish and update the speed of the robot
    node_publisher.publish(twist);
}

int main(int argc, char *argv[])
{
    // Load Gazebo
    gazebo::client::setup(argc, argv);
    // Ros init
    ros::init(argc, argv, "remote_controller");
    ros::NodeHandle node_hanlde;

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", gazebo_contact_read);

    // Init publisher and subscribers
    // auto pos_sub = node_hanlde.subscribe(turtle_name+"/tf", 10, &poseCallback);
    //Publishers - cmd_vel is controlling the robot direction
    node_publisher = node_hanlde.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    node_score_pub = node_hanlde.advertise<std_msgs::Float32MultiArray>("score_and_time", 10);
    set_model_state_client = node_hanlde.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    //Subscribers - to collect the published dataset
    auto shimmer_sub = node_hanlde.subscribe("shimmer_imu_pub", 10, control_robot);
    auto sensor_sub = node_hanlde.subscribe("scan", 10, sensorHandler);
    auto odom_values = node_hanlde.subscribe("odom", 10, odomHandler);
    auto stress_result = node_hanlde.subscribe("stress_publisher", 10, stress_handler);

    // gazebo::gui::Events::follow("turtlebot3_burger_controller");
    ROS_INFO("Robot_controller has been started ...");
    // this function will be used to wait until the
    // publisher is killed
    ros::spin();
}