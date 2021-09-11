#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <async_comm/serial.h>
#include <async_comm/util/message_handler_ros.h>
#include "shimmer3.hpp"
#include "filter.hpp"

// Filter declerations
using vector_cast = std::vector<double>;
// constexpr double LPF_CORNER_FREQ_HZ = 2.2;
// constexpr double HPF_CORNER_FREQ_HZ = 0.65;
// Shimmer3::Filter LPF_PPG(Shimmer3::LOW_PASS, SHIMER_SAMPLE_RATE, vector_cast{LPF_CORNER_FREQ_HZ});
// Shimmer3::Filter HPF_PPG(Shimmer3::HIGH_PASS, SHIMER_SAMPLE_RATE, vector_cast{HPF_CORNER_FREQ_HZ});

bool wait_serial_responce = false;
bool setup_mode = true;

Shimmer3::Shimmer3_packets received_packs{0};

Shimmer3::PPGtoHRAlgorithm PPG2HR(SHIMER_SAMPLE_RATE, 1, 1);
// packet types: 0x08-> Set sensor command and 0x05 -> set sampling rate command (util.py)
constexpr uint8_t command_params[] = {Shimmer3::SET_SENSORS_COMMAND, 0xC4, 0x01, 0x00};                      // analog accel, ppg, gsr, MPU9150 gyro
constexpr uint8_t sampling_rate_command[] = {Shimmer3::SET_SAMPLING_RATE_COMMAND,                            // Arrange shimmer's sample rate according to SHIMER_SAMPLE_RATE
                                             Shimmer3::arrange_sample_rate<SHIMER_SAMPLE_RATE>::val_first,   // arrange the bytes according to SHIMER_SAMPLE_RATE
                                             Shimmer3::arrange_sample_rate<SHIMER_SAMPLE_RATE>::val_second}; // 51.2Hz (32768/640=51.2Hz: 640 -> 0x0280;
                                                                                                             // has to be done like this for alignment reasons.)

uint8_t payload[Shimmer3::PAYLOAD_SIZE]{0};

// ROS definition
ros::Publisher num_publisher;

void callback(const uint8_t *const buf, size_t len)
{
    if (setup_mode)
    {
        if (*buf == 0xFF)
            wait_serial_responce = true;
        else
        {
            wait_serial_responce = false;
        }
    }
    else
    {
        //check the upcoming data length (For eliminating incorrect dataset)
        static bool enter{true};
        if (len == Shimmer3::PAYLOAD_SIZE && enter)
        {
            enter = false;
            size_t indx{0};
            received_packs.packet_type = buf[indx];                                                  // 0
            received_packs.time_stamp = (buf[++indx] + (buf[++indx] * 256) + (buf[++indx] * 65536)); // 1
            received_packs.Analog_Accel_x = *((uint16_t *)(&buf[(++indx)]));                         // 4
            received_packs.Analog_Accel_y = *((uint16_t *)(&buf[++(++indx)]));                       // 6
            received_packs.Analog_Accel_z = *((uint16_t *)(&buf[++(++indx)]));
            received_packs.PPG = *((uint16_t *)(&buf[++(++indx)]));
            received_packs.GSR = *((uint16_t *)(&buf[++(++indx)]));
            received_packs.mpu9150_gyro_x = *((int16_t *)(&buf[++(++indx)]));
            received_packs.mpu9150_gyro_y = *((int16_t *)(&buf[++(++indx)]));
            received_packs.mpu9150_gyro_z = *((int16_t *)(&buf[++(++indx)]));

            /* 
               Depending on the implementation, we can directly convert ppg to
               heart-rate or other processing processes like filtering and then 
               publish them. But we can also leave these processes to the python scripts
               after publishing the collected raw values.
            */

            auto cal_PPG = Shimmer3::calibrate_ppg(received_packs.PPG); // We can also use Raw ppg signal for ppg to hr
            // double dataFilteredLP = LPF_PPG.filterData(received_packs.PPG);
            // double dataFilteredHP = HPF_PPG.filterData(dataFilteredLP);
            auto tmp = Shimmer3::CalibrateTimeStamp(received_packs.time_stamp);
            auto hr = PPG2HR.ppg_to_hr(received_packs.PPG, tmp);

            // Print all the received dataset to check them
            // std::cout << received_packs.time_stamp << "   ";
            // std::cout << received_packs.Analog_Accel_x << "   ";
            // std::cout << received_packs.Analog_Accel_y << "   ";
            // std::cout << received_packs.Analog_Accel_z << "   ";
            // std::cout << received_packs.PPG << "   ";
            // std::cout << cal_PPG << "   ";
            // std::cout << hr.first << "   ";   // heart rate
            // std::cout << PPG2HR._get_hrv() << "   "; // heart rate variability
            // std::cout << received_packs.GSR << "   ";
            // Shimmer3::_active_gsr_mu = Shimmer3::GSR_SKIN_CONDUCTANCE;
            // std::cout << Shimmer3::calibrate_gsr(received_packs.GSR) << "   ";
            // Shimmer3::_active_gsr_mu = Shimmer3::GSR_SKIN_RESISTANCE;
            // std::cout << Shimmer3::calibrate_gsr(received_packs.GSR) << "   ";
            // std::cout << received_packs.mpu9150_gyro_x << "   ";
            // std::cout << received_packs.mpu9150_gyro_y << "   ";
            // std::cout << received_packs.mpu9150_gyro_z ;
            // std::cout << std::endl;

            std_msgs::Float32MultiArray array;

            array.data.push_back(received_packs.Analog_Accel_x); // 0
            array.data.push_back(received_packs.Analog_Accel_y); // 1
            array.data.push_back(received_packs.Analog_Accel_z); // 2
            array.data.push_back(received_packs.time_stamp);
            array.data.push_back(received_packs.PPG);
            array.data.push_back(cal_PPG);
            array.data.push_back(hr.first);          // heart rate
            array.data.push_back(PPG2HR._get_hrv()); // heart rate variability
            array.data.push_back(received_packs.GSR);
            array.data.push_back(Shimmer3::calibrate_gsr_v2(received_packs.GSR));
            Shimmer3::_active_gsr_mu = Shimmer3::GSR_SKIN_CONDUCTANCE;
            array.data.push_back(Shimmer3::calibrate_gsr(received_packs.GSR));
            Shimmer3::_active_gsr_mu = Shimmer3::GSR_SKIN_RESISTANCE;
            array.data.push_back(Shimmer3::calibrate_gsr(received_packs.GSR));

            // Publish the pushed dataset
            num_publisher.publish(array);
            enter = true;
        }
    }
}

void wait_for_resp()
{
    while (!wait_serial_responce)
        ;

    wait_serial_responce = false;
}

template <std::size_t N, class T>
constexpr std::size_t arr_size(T (&)[N]) { return N; }

template <typename my_serial>
void shimmer3_setup(my_serial &ser)
{

    ser.send_bytes(command_params, arr_size(command_params));
    wait_for_resp();

    ser.send_bytes(sampling_rate_command, arr_size(sampling_rate_command));
    wait_for_resp();

    ser.send_bytes(ENUM_CAST(Shimmer3::START_STREAMING_COMMAND), 1);
    wait_for_resp();

    setup_mode = false;
}

int main(int argc, char **argv)
{
    /* While running this node, enter the available com port 
       after the ros run command e.g 
       rosrun my_example_pkg publisher /dev/rfcomm0
       Namely:
       rosrun workspace_name node_name com_port_name  */

    // initialize
    char *port_name;
    if (argc < 2)
    {
        std::cout << "USAGE: " << argv[0] << " PORT\n";
        return 1;
    }
    else
    {
        std::cout << "Using port " << argv[1] << "\n";
        port_name = argv[1];
    }

    // Open serial port
    async_comm::Serial serial(port_name, Shimmer3::BAUD_RATE);

    serial.register_receive_callback(&callback);

    if (!serial.init())
    {
        std::cout << "Failed to initialize serial port\n";
        return 2;
    }
    else
    {
        std::cout << "Successfull connection ! \n";
    }

    shimmer3_setup(serial);

    ros::init(argc, argv, "pub_rpm");

    ros::NodeHandle node_hanlde;

    // We need to define the advertise type, if the data to be sent is Float32MultiArray format
    // the advertise function template should be defined as below.
    num_publisher = node_hanlde.advertise<std_msgs::Float32MultiArray>("shimmer_imu_pub", 10);
    // Inform the user that the Ros node has been started
    ROS_INFO("My Num_Publisher Node Started...");

    // Wait the upcoming data from the serial port (Bluetooth port)
    ros::spin();
    return 0;
}
