#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <unordered_map>
#include <string>

// If automatic controlling mode is demanded
#define AUTOMATIC_MODE 0

// Imu sensors limits
#define UPPER_BOUNDARY 2150
#define LOWER_BOUNDARY 1950

// Stress threshold
#define STRESS_THRESHOLD 0.5

#if AUTOMATIC_MODE

#define CLOSEST_DISTANCE 0.7
#define SIDE_DISTANCE 0.5
#define TURNING_DEGREE 5.0

#else

#define CLOSEST_DISTANCE 0.4
#define SIDE_DISTANCE 0.4
#define TURNING_DEGREE 30.0
#define LIMIT_SPEED_DIST 0.8

#endif

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define LINEAR_VELOCITY 0.25
#define ANGULAR_VELOCITY 0.5

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

// Variables
double escape_range_ = TURNING_DEGREE * DEG2RAD;
double check_forward_dist_ = CLOSEST_DISTANCE;
double check_side_dist_ = SIDE_DISTANCE;

double scan_data_[3] = {0.0, 0.0, 0.0};

double tb3_pose_{0.0};
double prev_tb3_pose_{0.0};

// Init variables
float speed(0.1); // Linear velocity (m/s)
float linear_gear{1};
float turn(1.0); // Angular velocity (rad/s)
float min_point{0.0};

ros::Publisher node_publisher;

// Create Twist message
geometry_msgs::Twist twist;

bool is_obstacle{false};
int obstacle_degree{-1};
float obstacle_distance{-1.0f};

bool is_stressful{false};


void stress_handler(const std_msgs::Float32::ConstPtr &received_stress)
{
/*  
    The GSR and PPG values will be processed in this function, and 
    it will be determined whether the person gets stressed or not
*/
    std::cout << received_stress->data << "   ";
    if (received_stress->data >= STRESS_THRESHOLD)
        is_stressful = true;
    else
        is_stressful = false;

    std::cout << std::boolalpha << is_stressful << std::endl;
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
    auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    obstacle_distance = *min_it;

    // Check is there any possible obstacle
    for (size_t degree = 0; degree < msg->ranges.size(); ++degree)
    {
        if (msg->ranges[degree] < CLOSEST_DISTANCE)
        { // Obstacle detected
            is_obstacle = true;
            obstacle_degree = static_cast<int>(degree);
            // std::cout << msg->ranges[degree] << std::endl;
            break;
        }
        else
        { // No any obstacle detected
            is_obstacle = false;
            obstacle_degree = -1;
        }
    }

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

void updatecommandVelocity(double linear, double angular)
{
    twist.linear.x = linear;
    twist.angular.z = angular;
}

bool obstacle_avoidance()
{
    static uint8_t turtlebot3_state = 0;

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
                turtlebot3_state = TB3_DRIVE_FORWARD;
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

template <typename S>
void update_linear_gear(S sensor)
{

    if (sensor > UPPER_BOUNDARY && sensor < UPPER_BOUNDARY + 100)
    {
        linear_gear = 2;
    }
    else if (sensor > UPPER_BOUNDARY + 100 && sensor < UPPER_BOUNDARY + 200)
    {
        linear_gear = 4;
    }
    else if (sensor > UPPER_BOUNDARY + 200 && sensor < UPPER_BOUNDARY + 300)
    {
        linear_gear = 6;
    }
    else if (sensor > UPPER_BOUNDARY + 300 && sensor < UPPER_BOUNDARY + 400)
    {
        linear_gear = 8;
    }
    else if (sensor > UPPER_BOUNDARY + 400 && sensor < UPPER_BOUNDARY + 500)
    {
        linear_gear = 10;
    }
    else
    {
        linear_gear = 10;
    }
}

/*  This function is used for control the robot with using IMU sensor values
 *  Besides, the person's physiological parameters will be captured to figure out 
 *  the stressful situations and in that circumstances, if the robot in danger, 
 *  we can intervene to the robot to avoid unintended consequences like accidents.
 */
void control_robot(const std_msgs::Float32MultiArray::ConstPtr &array)
{
#if AUTOMATIC_MODE

    obstacle_avoidance();

#else
    // if the user gets stressed, check around to see whether there is an obstacle
    if (is_obstacle && is_stressful)
    {
        if (obstacle_distance < LIMIT_SPEED_DIST)
            linear_gear = 2; // limit the speed under certain distance

        // if obstacle is detected
        if (obstacle_degree != -1)
        {
            // Run Automatic obstacle avoidance algorithm
            obstacle_avoidance();
        }
    }

    else
    {
        // Use the upcomming shimmer3 imu values to control the robot
        // Linear acceleration - pitch movement of shimmer sensor
        update_linear_gear(array->data[0]);

        if (array->data[0] > UPPER_BOUNDARY)
        {
            twist.linear.x = speed * linear_gear;
            twist.linear.y = 0;
            twist.linear.z = 0;
        }
        else if (array->data[0] < LOWER_BOUNDARY)
        {
            twist.linear.x = -speed * 5;
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
        if (array->data[1] > UPPER_BOUNDARY)
        {
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = -turn;
        }
        else if (array->data[1] < LOWER_BOUNDARY)
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

#endif

    // Publish and update the speed of the robot
    node_publisher.publish(twist);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "remote_controller");
    ros::NodeHandle node_hanlde;

    // Init publisher and subscribers
    //Publishers - cmd_vel is controlling the robot direction
    node_publisher = node_hanlde.advertise<geometry_msgs::Twist>("cmd_vel", 10);
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