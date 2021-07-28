#ifndef _SHIMMER3_HPP_
#define _SHIMMER3_HPP_

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <utility> // make_pair

#define ENUM_CAST(val) &static_cast<const uint8_t &>(val)

#define SHIMER_SAMPLE_RATE 64 // 51.2, 102.4, 204.8, 256, 512

namespace Shimmer3
{
    // Required commands for this robot control application
    enum Shimmer_commands : uint8_t
    {
        INQUIRY_COMMAND = 0x01,
        SET_SAMPLING_RATE_COMMAND = 0x05,
        START_STREAMING_COMMAND = 0x07,
        SET_SENSORS_COMMAND = 0x08,
        STOP_STREAMING_COMMAND = 0x20,
        SET_RWC_COMMAND = 0x8F,
    };

    constexpr const auto BAUD_RATE = 460800; // 9600, 19200, 38400, 57600, 230400, 460800, 921600 
    constexpr const size_t PAYLOAD_SIZE = 20;

    enum Gsr_skin_types
    {
        GSR_SKIN_CONDUCTANCE,
        GSR_SKIN_RESISTANCE
    };

    Gsr_skin_types _active_gsr_mu{GSR_SKIN_CONDUCTANCE};

    struct Shimmer3_packets
    {
        uint8_t packet_type;
        uint32_t time_stamp;
        uint16_t Analog_Accel_x;
        uint16_t Analog_Accel_y;
        uint16_t Analog_Accel_z;
        uint16_t GSR;
        uint16_t PPG;
        int16_t mpu9150_gyro_x;
        int16_t mpu9150_gyro_y;
        int16_t mpu9150_gyro_z;
    };

    template <uint16_t T>
    struct arrange_sample_rate
    {
        static constexpr const uint16_t value = 32768 / T;
        static constexpr const uint8_t val_first = value & 0xFF;
        static constexpr const uint8_t val_second = value >> 8;
    };

    template <typename R_data>
    auto calibrate_ppg(const R_data raw_data) -> float
    {
        /*
        Calibrate the PPG data.
        : param raw_data : the raw data of the PPG sensor
        : return : the calibrated data */

        return raw_data * 3000.0 / 4095.0;
    }

    template <typename T>
    auto calibrate_gsr_v2(const T &GSR_raw)
    {

        auto Range = ((GSR_raw >> 14) & 0xff); // upper two bits
        float Rf{0};

        if (Range == 0)
            Rf = 40.2; //kohm
        else if (Range == 1)
            Rf = 287.0; //kohm
        else if (Range == 2)
            Rf = 1000.0; //kohm
        else if (Range == 3)
            Rf = 3300.0; //kohm

        auto gsr_to_volts = (GSR_raw & 0x3fff) * (3.0 / 4095.0);
        auto GSR_ohm = Rf / ((gsr_to_volts / 0.5) - 1.0);

        return GSR_ohm;
    }

    template <typename G_raw>
    auto calibrate_gsr(const G_raw &raw_data)
    {

        auto range_settings = (raw_data >> 14) & 0x3; // take only two upper bits
        auto adc_value = raw_data & 0xfff;            // take only twelve lower bits
        auto r_f = 3300000;                           // with auto_range -> range 3 -> 3300000
        if (range_settings == 0)
            r_f = 40200;
        else if (range_settings == 1)
            r_f = 287000;
        else if (range_settings == 2)
            r_f = 1000000;
        else if (range_settings == 3)
            r_f = 3300000;
        auto gsr_to_volts = adc_value * (3.0 / 4095.0);
        auto gsr_ohm = r_f / ((gsr_to_volts / 0.5) - 1.0);

        if (_active_gsr_mu == GSR_SKIN_CONDUCTANCE)
        {
            auto skin_conductance = (1 / gsr_ohm) * 1000000; // microSiemiens - Skin Conductance
            return skin_conductance;
        }
        else if (_active_gsr_mu == GSR_SKIN_RESISTANCE)
        {
            return gsr_ohm / 1000; // kOhm - Skin Resistance
        }
        else
        {
            // print("calibrate_gsr -> WARNING: Not supported yet...returned 'None'")
            return 0.0;
        }
    }

    // This function and calib_time_stamp have been from ANSIA-A-Nural_system python scripts
    long calibrate_timestamp_time_elapsed(long raw_data, long _first_local_timestamp_of_a_stream)
    {
        /*Transform the timestamp in seconds elapsed from the first timestamp of the stream.

        :param raw_data: current timestamp
        :return: seconds elapsed from the first packet of the stream */
        return raw_data / 32768 - _first_local_timestamp_of_a_stream / 32768;
    }

    long calib_time_stamp(long timestamp)
    {
        static long previous_timestamp{-1};
        static long first_local_timestamp_of_a_stream{-1};
        static long first_unix_timestamp_of_a_stream{-1};
        static long previous_calibrated_timestamp{-1};
        static int clock_overflows{0};

        std::vector<long> packet(3);

        if (previous_timestamp != -1 && timestamp < previous_timestamp)
            clock_overflows++;
        previous_timestamp = timestamp;

        timestamp += clock_overflows * 16777216;

        if (first_local_timestamp_of_a_stream == -1)
            first_local_timestamp_of_a_stream = timestamp;
        if (first_unix_timestamp_of_a_stream == -1)
            first_unix_timestamp_of_a_stream = std::time(nullptr); // should be time in second

        packet.push_back(timestamp);
        packet.push_back(first_unix_timestamp_of_a_stream);
        auto calibrated_timestamp = first_unix_timestamp_of_a_stream +
                                    calibrate_timestamp_time_elapsed(timestamp, first_local_timestamp_of_a_stream);
        std::cout << "calibrated time: " << calibrated_timestamp << std::endl;
        packet.push_back(calibrated_timestamp);

        if (previous_calibrated_timestamp != -1)
        {
            auto delta = calibrated_timestamp - previous_calibrated_timestamp;
            if (delta > (1 / SHIMER_SAMPLE_RATE))
                std::cout << "PACKET LOSS! Registered delta: " << delta;
        }

        previous_calibrated_timestamp = calibrated_timestamp;

        return calibrated_timestamp;
    }

    // This calibration function is from shimmer3 C# code provided by the vendor
    double CalibrateTimeStamp(double timeStamp)
    {
        static double LastReceivedTimeStamp = 0;
        static double CurrentTimeStampCycle = 0;
        static double LastReceivedCalibratedTimeStamp = -1;
        static double TimeStampPacketRawMaxValue = 16777216; // 16777216 or 65536
        static bool FirstTimeCalTime = true;
        static double CalTimeStart = 0;
        static double SamplingRate = SHIMER_SAMPLE_RATE;
        static double PacketLossCount = 0;
        static double ADCRawSamplingRateValue = SHIMER_SAMPLE_RATE;
        static double PacketReceptionRate = 100;

        //first convert to continuous time stamp
        double calibratedTimeStamp = 0;
        if (LastReceivedTimeStamp > (timeStamp + (TimeStampPacketRawMaxValue * CurrentTimeStampCycle)))
        {
            CurrentTimeStampCycle = CurrentTimeStampCycle + 1;
        }

        LastReceivedTimeStamp = (timeStamp + (TimeStampPacketRawMaxValue * CurrentTimeStampCycle));
        calibratedTimeStamp = LastReceivedTimeStamp / 32768 * 1000; // to convert into mS
        if (FirstTimeCalTime)
        {
            FirstTimeCalTime = false;
            CalTimeStart = calibratedTimeStamp;
        }
        if (LastReceivedCalibratedTimeStamp != -1)
        {
            double timeDifference = calibratedTimeStamp - LastReceivedCalibratedTimeStamp;
            double clockConstant = 32768; // for SHIMMER3 it should be 32768

            double expectedTimeDifference = (1 / SamplingRate) * 1000; //in ms
            double adjustedETD = expectedTimeDifference + (expectedTimeDifference * 0.1);

            if (timeDifference > adjustedETD)
            {
                //calculate the estimated packet loss within that time period
                int numberOfLostPackets = ((int)std::ceil(timeDifference / expectedTimeDifference)) - 1;
                PacketLossCount = PacketLossCount + numberOfLostPackets;
                long mTotalNumberofPackets = (long)((calibratedTimeStamp - CalTimeStart) / (1 / (clockConstant / ADCRawSamplingRateValue) * 1000));
                mTotalNumberofPackets = (long)((calibratedTimeStamp - CalTimeStart) / expectedTimeDifference);
                PacketReceptionRate = (double)((mTotalNumberofPackets - PacketLossCount) / (double)mTotalNumberofPackets) * 100;
            }
        }

        LastReceivedCalibratedTimeStamp = calibratedTimeStamp;
        return calibratedTimeStamp - CalTimeStart; // make it start at zero
    }

    class PPGtoHRAlgorithm
    {
    public:
        template <typename S_rate, typename N_average, typename L_est>
        PPGtoHRAlgorithm(S_rate sampling_rate, N_average number_of_beats_to_average, L_est use_last_estimate)
        {

            this->_peak_ppg_data.clear();
            this->_peak_timestamps.clear();
            this->_ppg_data.clear();
            this->_timestamps.clear();

            this->_value_mean = 0.0;
            this->_value_peak = 0.0;

            this->_slope = 0.0;
            this->_threshold = 0.0;
            this->_climbing = false;

            this->_last_known_hr = -60.0;
            this->_heart_rate = INVALID_RESULT; //check type
            this->hrv_value_ms = INVALID_RESULT;

            this->_pulse_period = 0.0;
            this->_first_pass = true;

            this->_default_buffer_size = 2;
            this->_IBI_test_buffer_size = 3;
            this->_use_last_estimate = use_last_estimate; // check type
            this->_sampling_rate = sampling_rate;         // check type

            this->_hr_upper_limit = 215; // above 215 the HR is considered invalid
            this->_hr_lower_limit = 30;  // below 30 the HR is considered invalid

            this->_ppg_data_buffer_size = static_cast<int>(this->_sampling_rate * 2.0);
            // print("PPG Data Buffer: ", _ppg_data_buffer_size)

            this->_default_number_of_beats_to_average = 1;

            if (number_of_beats_to_average < 1)
            {
                this->_number_of_beats_to_average = 1;
            }
            else
            {
                this->_number_of_beats_to_average = number_of_beats_to_average;
            }

            this->_number_of_samples_since_peak = 0;

            this->_max_half_beat_interval = 300.0;

            _set_parameters(sampling_rate, number_of_beats_to_average, this->_default_buffer_size);
        }

        template <typename P_sample, typename T_sample>
        auto ppg_to_hr(P_sample ppg_sample, T_sample timestamp_sample)
        {
            this->_ppg_data.push_back(ppg_sample);
            this->_timestamps.push_back(timestamp_sample);
            if (this->_ppg_data.size() < this->_ppg_data_buffer_size)
            {
                //  We don't have enough ppg data to compute an Heart Rate
                return std::make_pair(INVALID_RESULT, INVALID_RESULT);
            }

            if (this->_first_pass)
            {
                const auto [min, max] = std::minmax_element(begin(this->_ppg_data), end(this->_ppg_data));
                this->_value_mean = *min;
                this->_value_peak = *max;
                this->_slope = 0.0;
                this->_threshold = 0.0;
                this->_climbing = true;
                _calculate_pulse_period();
                this->_first_pass = false;
            }

            auto hr = _compute_heart_rate();
            if (this->_peak_timestamps.size() > 2)
            {
                const size_t pos = this->_peak_timestamps.size();
                return std::make_pair(hr, (this->_peak_timestamps[pos - 1] - this->_peak_timestamps[pos - 2]));
            }
            else
            {
                return std::make_pair(hr, INVALID_RESULT);
            }
        }

        auto _compute_heart_rate()
        {
            if (this->_ppg_data.size() < this->_ppg_data_buffer_size)
                return this->_heart_rate;

            // Take only the last 'ppg_data_buffer_size' data
            for (int i = 0; i < (this->_ppg_data.size() - this->_ppg_data_buffer_size); ++i)
            {
                this->_ppg_data.erase(this->_ppg_data.begin());
                this->_timestamps.erase(this->_timestamps.begin());
            }

            auto new_peak_found = false;
            // Check if the threshold should be increased and check also if set climbing to True
            if (this->_ppg_data[this->_ppg_data_buffer_size - 1] > this->_ppg_data[this->_ppg_data_buffer_size - 2] &&
                this->_ppg_data[this->_ppg_data_buffer_size - 1] > this->_threshold)
            {
                this->_threshold = this->_ppg_data[this->_ppg_data_buffer_size - 1]; // threshold is updated
                this->_number_of_samples_since_peak = 0;
                this->_climbing = true;
            }
            else
            {
                // # If we were climbing but the last data is less than the penultimate so the penultimate was a peak
                if (this->_climbing)
                {
                    this->_peak_timestamps.push_back(this->_timestamps[this->_ppg_data_buffer_size - 2]);
                    this->_peak_ppg_data.push_back(this->_ppg_data[this->_ppg_data_buffer_size - 2]);
                    this->_value_peak = this->_ppg_data[this->_ppg_data_buffer_size - 2];
                    this->_value_mean = this->_ppg_data[std::distance(this->_ppg_data.begin(),
                                                                      std::min_element(this->_ppg_data.begin(), this->_ppg_data.end()))];
                    _calculate_pulse_period();
                    _calculate_slope();
                    new_peak_found = true;
                    this->_number_of_samples_since_peak = 1;
                }
                else
                {
                    // If we were not climbing we are going down so the peak has been exceeded by 'number_of_samples_since'
                    // (plus one now)
                    this->_number_of_samples_since_peak++;
                }

                _calculate_threshold();
                this->_climbing = false;
            }
            if (new_peak_found)
            {
                if (this->_heart_rate != INVALID_RESULT)
                {
                    _remove_false_peaks();
                }
                auto number_peaks = this->_peak_ppg_data.size();

                auto min_samples_needed = this->_number_of_beats_to_average + 2;
                if (this->_IBI_test_buffer_size + 1 > this->_number_of_beats_to_average + 2)
                    min_samples_needed = this->_IBI_test_buffer_size + 1;

                if (number_peaks < min_samples_needed)
                {
                    this->_heart_rate = INVALID_RESULT;
                }
                else
                {
                    _calculate_heart_rate(number_peaks);
                    for (int j = 0; j < (this->_peak_ppg_data.size() - min_samples_needed); ++j)
                    {
                        this->_peak_ppg_data.erase(this->_peak_ppg_data.begin());
                        this->_peak_timestamps.erase(this->_peak_timestamps.begin());
                    }
                }
            }
            else
            {
                this->hrv_value_ms = INVALID_RESULT;
            }
            return this->_heart_rate;
        }

        void _calculate_slope()
        {
            this->_slope = -std::abs(2.0 * (this->_value_peak - this->_value_mean) / (4.5 * this->_pulse_period));
        }

        void _calculate_threshold()
        {
            this->_threshold = this->_value_peak + this->_number_of_samples_since_peak * this->_slope;
        }

        void _remove_false_peaks()
        {
            auto half_beat_size = 1000.0 / abs(this->_last_known_hr / 60.0) * 0.5;
            if (half_beat_size > this->_max_half_beat_interval)
                half_beat_size = this->_max_half_beat_interval;

            size_t index = 0;
            bool continue_loop = true;

            while (continue_loop)
            {
                if ((this->_peak_ppg_data.size() < 2) || (index >= this->_peak_ppg_data.size() - 2))
                {
                    continue_loop = false;
                }
                else
                {
                    auto timestamp_diff = this->_peak_timestamps[index + 1] - this->_peak_timestamps[index];
                    if (timestamp_diff < half_beat_size)
                    {
                        auto it_ppg = this->_peak_ppg_data.begin();
                        auto it_time = this->_peak_timestamps.begin();
                        if (this->_peak_ppg_data[index] > this->_peak_ppg_data[index + 1])
                        {
                            this->_peak_ppg_data.erase(it_ppg + index + 1);
                            this->_peak_timestamps.erase(it_time + index + 1);
                        }
                        else
                        {
                            this->_peak_ppg_data.erase(it_ppg + index);
                            this->_peak_timestamps.erase(it_time + index);
                        }
                    }
                    else
                    {
                        index += 1;
                    }
                }
            }
        }

        void _calculate_heart_rate(int number_peaks)
        {
            std::vector<int> inter_beat_intervals;

            auto subs = [](auto &l, auto &r)
            { return (l - r); };

            auto start1 = this->_peak_timestamps.begin() + number_peaks - this->_number_of_beats_to_average - 1;
            auto end1 = this->_peak_timestamps.begin() + number_peaks - 1;

            std::transform(start1, end1, start1 - 1, std::back_inserter(inter_beat_intervals), subs);

            std::vector<long> ibi_buffer;

            for (int i = 0; i < this->_number_of_beats_to_average; ++i)
            {
                ibi_buffer.push_back(inter_beat_intervals[i]);
            }

            this->hrv_value_ms = *(ibi_buffer.end() - 1); // Update hrv value

            start1 = this->_peak_timestamps.begin() + number_peaks - this->_IBI_test_buffer_size;
            end1 = this->_peak_timestamps.begin() + number_peaks;

            std::vector<long> ibi_test_buffer;

            std::transform(start1, end1, start1 - 1, std::back_inserter(ibi_test_buffer), subs);

            std::vector<float> hr_test_buffer;
            for (int j = 0; j < this->_IBI_test_buffer_size; ++j)
            {
                hr_test_buffer.push_back(1.0 / ibi_test_buffer[j]);
            }

            const auto [min, max] = std::minmax_element(begin(hr_test_buffer), end(hr_test_buffer));

            float hr_median = _get_median(hr_test_buffer);
            auto hr_range = *max - *min;

            if (hr_median * 0.3 < hr_range)
            {
                if (this->_use_last_estimate)
                    this->_heart_rate = this->_last_known_hr;
                else
                    this->_heart_rate = INVALID_RESULT;
            }
            else
            {
                auto avg_interval = _calculate_mean(ibi_buffer);
                if (avg_interval == 0.0)
                    this->_heart_rate = INVALID_RESULT;
                else
                    this->_heart_rate = 60000.0 / avg_interval;

                if (this->_heart_rate > this->_hr_upper_limit || this->_heart_rate < this->_hr_lower_limit)
                {
                    if (this->_use_last_estimate)
                        this->_heart_rate = this->_last_known_hr;
                    else
                        this->_heart_rate = INVALID_RESULT;
                }
                else
                    this->_last_known_hr = this->_heart_rate;
            }
        }

        int _get_hrv() const
        {
            return this->hrv_value_ms;
        }

        void _calculate_pulse_period()
        {
            this->_pulse_period = this->_sampling_rate / std::abs(this->_last_known_hr / 60.0);
        }

        template <typename T>
        auto _calculate_mean(T &lst) -> float
        {
            auto s = 0.0;
            auto num_elements = lst.size();

            if (!num_elements)
                return 0; // TODO:

            for (auto &element : lst)
                s += element;

            return s / num_elements;
        }

        template <typename T>
        auto _get_median(T &lst) -> float
        {
            std::sort(lst.begin(), lst.end());
            auto num_elements = lst.size();
            auto middle = static_cast<int>(num_elements / 2);
            float median;
            if (num_elements % 2 == 0)
            {
                auto median_a = lst[middle];
                auto median_b = lst[middle - 1];
                median = (median_a + median_b) / 2.0;
            }
            else
            {
                median = lst[middle + 1];
            }

            return median;
        }

        void _reset_member_variables()
        {
            this->_climbing = true;
            this->_threshold = 0.0;
            this->_hr_upper_limit = 215;
            this->_hr_lower_limit = 30;
            this->_last_known_hr = -60.0;
            this->_heart_rate = INVALID_RESULT;

            this->_value_peak = 0.0;
            this->_value_mean = 0.0;
            this->_number_of_samples_since_peak = 0;

            this->_slope = 0.0;
            this->_threshold = 0.0;
            this->_climbing = true;
            this->_first_pass = true;

            this->_ppg_data.clear();
            this->_timestamps.clear();
        }

        template <typename S_rate, typename N_average, typename B_sz>
        void _set_parameters(S_rate sampling_rate, N_average num_beats_to_average, B_sz buffer_size)
        {
            _reset_member_variables();
            this->_sampling_rate = sampling_rate;
            if (buffer_size <= 0)
                buffer_size = 1;
            this->_ppg_data_buffer_size = static_cast<int>(this->_sampling_rate * 2.0);
            this->_number_of_beats_to_average = num_beats_to_average;
            if (this->_number_of_beats_to_average < 1)
                this->_number_of_beats_to_average = 1;
        }

    private:
        static constexpr const float INVALID_RESULT = 60;

        std::vector<float> _peak_ppg_data;
        std::vector<float> _peak_timestamps;
        std::vector<float> _ppg_data;
        std::vector<float> _timestamps;

        float _value_mean;
        float _value_peak;

        float _slope;
        float _threshold;
        bool _climbing;

        float _last_known_hr;
        float _heart_rate;

        float _pulse_period;
        bool _first_pass;

        float hrv_value_ms;

        size_t _default_buffer_size;
        size_t _IBI_test_buffer_size;
        float _use_last_estimate;
        float _sampling_rate;

        uint16_t _hr_upper_limit;
        uint16_t _hr_lower_limit;
        uint16_t _ppg_data_buffer_size;

        float _default_number_of_beats_to_average;

        float _number_of_beats_to_average;
        float _number_of_samples_since_peak;
        float _max_half_beat_interval;
    };

}
#endif //_SHIMMER3_HPP_