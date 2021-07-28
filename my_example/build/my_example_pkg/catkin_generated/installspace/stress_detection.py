import sys
import rospy
import datetime
import queue
import time
import math
from rospy.client import spin
from rospy.core import rospyinfo
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from scipy.signal import filtfilt, butter
import pickle
import warnings
import neurokit2 as nk
import tensorflow as tf
from models import simple_deep_esn_classifier  # for normal running
from models.ppg_to_hr import PPGtoHRAlgorithm
# from src.models import simple_deep_esn_classifier # for roscore
import numpy as np
from enum import IntEnum

SAMPLE_RATE = 64.0
WINDOW_LIMIT = 30  # second
PREDICTION_FREQUENCY = 1  # second
STRESS_THRESHOLD = 0.5

# Recieved packet indexes


class Shimmer_pack(IntEnum):
    Analog_Accel_x = 0
    Analog_Accel_y = 1
    Analog_Accel_z = 2
    time_stamp = 3
    PPG_RAW = 4
    PPG_CALIBRATED = 5
    heart_rate = 6
    heart_rate_variability = 7
    GSR_RAW = 8
    GSR_CALIBRATED = 9
    GSR_SKIN_CONDUCTANCE = 10
    GSR_SKIN_RESISTANCE = 11


def warn(*args, **kwargs):
    pass


warnings.warn = warn


def downsampling_mean(data, start_freq, end_freq):
    """
    This method resamples a data from the frequency 'start_freq' to 'end_freq' with the "mean" method.

    :param data: data to resample
    :param start_freq: start frequency
    :param end_freq: end frequency
    :return: resampled data
    """
    if start_freq < end_freq:
        print("You should use this function to downsample a timeseries...")
        return None
    else:
        n_to_collapse = int(start_freq / end_freq)
        counter = 0
        s = 0
        new_data = []
        for i in range(len(data)):
            s += data[i]
            counter += 1
            if counter == n_to_collapse:
                new_data.append(s / n_to_collapse)
                s = 0
                counter = 0

        return np.array(new_data)


def downsampling_last(data, start_freq, end_freq):
    """
    This method resamples a data from the frequency 'start_freq' to 'end_freq' with the "last" method.

    :param data: data to resample
    :param start_freq: start frequency
    :param end_freq: end frequency
    :return: resampled data
    """
    if start_freq < end_freq:
        print("You should use this function to downsample a timeseries...")
        return None
    else:
        n_to_collapse = int(start_freq / end_freq)
        new_data = []
        for i in range(len(data)):
            if i % n_to_collapse == 0:
                new_data.append(data[i])

        return np.array(new_data)


# This function computes the coefficient of the IIR filter
def butter_lowpass(cutoff, fs, order=5):
    """
    This function computes the coefficient of the IIR low pass filter of cutoff 'cutoff' for a signal that has
    frequency 'fs' and with order (of the filter) 'order'.

    :param cutoff: cutoff frequency of the lowpass filter
    :param fs: frequency of the signal to filter
    :param order: order of the filter
    :return: coefficients b and a of the IIR filter
    """
    nyq = 0.5 * fs
    normalized_cutoff = cutoff / nyq
    b, a = butter(order, normalized_cutoff, btype='low',
                  analog=False, output='ba')
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    """
    This function create an IIR low pass filter and applies it to the signal.

    :param data: signal to filter
    :param cutoff: cutoff frequency of the filter
    :param fs: frequency of the signal to filter
    :param order: order of the filter
    :return: filtered data
    """
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data, axis=0)
    return y


def calibrate_timestamp_time_elapsed(raw_data, _first_local_timestamp_of_a_stream):
    """
    Transform the timestamp in seconds elapsed from the first timestamp of the stream.
    :param raw_data: current timestamp
    :return: seconds elapsed from the first packet of the stream
    """
    cal_first_timestamp_of_a_stream = _first_local_timestamp_of_a_stream / 32768
    return [raw_data / 32768 - cal_first_timestamp_of_a_stream]


# global definitions for the time function
_previous_timestamp = -1
_first_local_timestamp_of_a_stream = -1
_first_unix_timestamp_of_a_stream = -1
_clock_overflows = 0
_previous_calibrated_timestamp = None


def time_syncer(time_stmp):

    global _previous_timestamp
    global _first_local_timestamp_of_a_stream
    global _first_unix_timestamp_of_a_stream
    global _clock_overflows
    global _previous_calibrated_timestamp

    local_pack = []
    timestamp = time_stmp
    # Check for overflows
    if _previous_timestamp != -1 and timestamp < _previous_timestamp:
        _clock_overflows += 1
    _previous_timestamp = timestamp

    timestamp = timestamp + _clock_overflows * 16777216

    if _first_local_timestamp_of_a_stream == -1:
        _first_local_timestamp_of_a_stream = timestamp
    if _first_unix_timestamp_of_a_stream == -1:
        _first_unix_timestamp_of_a_stream = time.time()  # in seconds

    local_pack.append(timestamp)
    local_pack.append(_first_unix_timestamp_of_a_stream)
    calibrated_timestamp = _first_unix_timestamp_of_a_stream + \
        calibrate_timestamp_time_elapsed(
            timestamp, _first_local_timestamp_of_a_stream)[0]
    local_pack.append(calibrated_timestamp)

    if _previous_calibrated_timestamp is not None:
        # Check for packet losses
        delta = calibrated_timestamp - _previous_calibrated_timestamp
        # if delta > (1 / SAMPLE_RATE): # 64 is sample rate don't forget to change it
        #     print("PACKET LOSS! Registered delta: ", delta)
    _previous_calibrated_timestamp = calibrated_timestamp

    return local_pack


# global definitions for overall predictions
_prediction_queue = queue.Queue()
_timestamps = []
_predictions_data = []
_accuracy = 0.0
_prec_accuracy = -1


def signal_processing_and_reservoir_computing(data=None, model=None, pub=None):
    """

    :param data: appended data to be processed
    :param model: Machine Learning classifier

    """
    global _prediction_queue
    global _timestamps
    global _predictions_data
    global _accuracy
    global _prec_accuracy

    timestamps = []
    hr = []
    eda = []

    # Remember that EDA and HR are at the same frequency, so here they have the same length
    for i in range(len(data['HR'])):
        hr.append(data['HR'][i][1])
        eda.append(data['EDA'][i][1])
        timestamps.append(data['EDA'][i][0])

    # Filtering EDA
    # print(eda)
    try:
        eda = butter_lowpass_filter(eda, 1, 64, order=4)
        # Applying NeuroKit2 on EDA
        signal, process = nk.eda_process(eda, sampling_rate=64)
        scr = signal['EDA_Phasic'].to_numpy()
        scr = scr.reshape((scr.shape[0], 1))
        scl = signal['EDA_Tonic'].to_numpy()
        scl = scl.reshape((scl.shape[0], 1))

        hr = np.array(hr).reshape((len(hr), 1))

        # Now we have to resample the signals to 4 Hz
        hr = downsampling_mean(hr, 64, 4)
        scr = downsampling_mean(scr, 64, 4)
        scl = downsampling_mean(scl, 64, 4)
        timestamps = downsampling_last(timestamps, 64, 4)

        # Let's standardize the data.
        means = []
        stds = []
        # If we have 30 seconds of data (30 seconds * 4 Hz)
        if len(scr) >= 4 * 30:
            try:
                # Let's see if the mean and stds are already computed and use them
                f = open("statistics.pkl", 'rb')
                statistics = pickle.load(f)
                means = statistics['means']
                stds = statistics['stds']
                f.close()
            except FileNotFoundError:
                # If this is the first time that a 'process_thread' have 30 seconds of data we have to
                # compute std and mean and use them for every data of the future
                statistics = {}
                means = np.array([np.mean(scr), np.mean(scl), np.mean(hr)])
                stds = np.array([np.std(scr), np.std(scl), np.std(hr)])
                statistics['means'] = means
                statistics['stds'] = stds
                f = open("statistics.pkl", 'wb')
                pickle.dump(statistics, f)
                f.close()
        # Now we should concatenate belong axis 1 to create the X
        # this is the order of the features (depends by the training process)
        X = np.concatenate((scr, scl, hr), axis=1)
        # print(X)
        if X.shape[0] >= 4 * 30:
            X = (X - means) / stds
            print("Mean of X: ", np.mean(X))
            print("Std of X: ", np.std(X))

        X = X.reshape((1, X.shape[0], X.shape[1]))

        predictions = model.predict(X)
        predictions = predictions.reshape((-1,))
        predictions = list(predictions)
        timestamps = list(timestamps)
        # We take only the last second of predictions
        if len(predictions) >= 4:
            predictions = predictions[-4:]
            timestamps = timestamps[-4:]

        mn = np.mean(predictions)
        mn = round(mn, 2)
        print("Instant Stress: {}".format(mn))
        _prediction_queue.put([timestamps, predictions])

        try:
            # Get the last prediction
            last_pred = _prediction_queue.get(block=False)
            # Each prediction has the form of [timestamps, predictions]
            _timestamps += last_pred[0]
            _predictions_data += last_pred[1]

            if len(_predictions_data) > 120:
                _predictions_data = _predictions_data[4:]
                _timestamps = _timestamps[4:]
            # In 'tmp_data' there will be predictions to which are applied the threshold function
            tmp_data = [
                1 if x > STRESS_THRESHOLD else 0 for x in _predictions_data]

            s = 0
            for element in tmp_data:
                if element == STRESS_THRESHOLD:
                    s += 1

            new_acc = round(s / len(tmp_data), 2)

            if _prec_accuracy != -1:
                new_acc = (new_acc + _prec_accuracy) / 2
            _accuracy = new_acc
            _prec_accuracy = new_acc

            mean = float(np.mean(_predictions_data))
            mean = round(mean, 2)
            print("Mean Stress: {}".format(mean))

            pub.publish(mean)  # Ros - publish the overall stress to the controller

        except queue.Empty:
            pass

    except ValueError:
        pass


def callback(msg, args):
    """

    This function will be called if the publisher node
    send new dataset

    """
    packet = msg.data
    model = args[0]
    reads = args[1]
    tmp_reads = args[2]
    PPG2HR = args[3]  # if ppgtohr used
    publisher = args[4]

    # publisher = args[3]

    # Here, this thread should read data and check when to make a prediction
    # We want to read 64 packets (1 second) before go ahead with the code execution
    # Note that PREDICTION FREQUENCY is the frequency of prediction expressed in seconds
    # 64 is the sample frequency of the Shimmer3 GSR+
    readden = len(tmp_reads['EDA'])
    # print(readden)
    if readden < int(PREDICTION_FREQUENCY * 64):
        # Synchronizing timestamp
        readden += 1  # total received packet bytes size
        # synced_timestamp = packet[Shimmer_pack.time_stamp]
        synced_timestamp = time_syncer(packet[Shimmer_pack.time_stamp])[2]
        timestamp = datetime.datetime.fromtimestamp(synced_timestamp).strftime(
            "%H:%M:%S.%f")[:-3]  # normal time format
        # print("synced_timestamp:{}    packet[2]:{}".format(synced_timestamp, packet[2]))

        # We use the calculated heart-rate by C++ code, or we can also calculate by using python script
        ppg = packet[Shimmer_pack.PPG_RAW]
        hr = PPG2HR.ppg_to_hr(ppg, synced_timestamp * 1000)[0]

        # ppg = packet[Shimmer_pack.PPG_CALIBRATED]
        # hr = packet[Shimmer_pack.heart_rate] # use calculated heart-rate by C++ code
        # print(hr)

        eda = packet[Shimmer_pack.GSR_SKIN_CONDUCTANCE]
        # print("HR:{}    EDA:{}    TimeStamp:{}".format(hr, eda, timestamp))
        tmp_reads['HR'].append([timestamp, hr])
        tmp_reads['EDA'].append([timestamp, eda])
    else:
        # Get the first 64 packets from reads
        how_much = int(PREDICTION_FREQUENCY * 64)
        to_append_HR = tmp_reads['HR'][0:how_much]
        to_append_EDA = tmp_reads['EDA'][0:how_much]
        tmp_reads['HR'] = tmp_reads['HR'][how_much:]
        tmp_reads['EDA'] = tmp_reads['EDA'][how_much:]

        # We want the last 30 second data (WINDOW_LIMIT = 30 and 64 is the sample rate)
        # Here we get rid of the first second data if the are more than 30 seconds of data
        if len(reads['HR']) >= int(64 * WINDOW_LIMIT) and len(reads['EDA']) >= int(64 * WINDOW_LIMIT):
            # clear all 30sec data frame
            reads['HR'] = reads['HR'][int(PREDICTION_FREQUENCY * 64):]
            reads['EDA'] = reads['EDA'][int(PREDICTION_FREQUENCY * 64):]
        # Here we append the new 1 second of data
        reads['HR'] = reads['HR'] + to_append_HR
        reads['EDA'] = reads['EDA'] + to_append_EDA

        signal_processing_and_reservoir_computing(reads, model, publisher)


def setup():

    # Retrieving model
    NUM_CLASSES = 2
    UNITS = 500
    LAYERS = 5
    SEQ_TO_SEQ = True
    CONCAT = True
    if LAYERS == 1:
        CONCAT = False
    LEAKY = 1
    SPECTRAL_RADIUS = 0.5
    INPUT_SCALING = 0.5

    params = np.load(
        '/home/furkan/Workspaces/my_example/src/my_example_pkg/src/models/model1.npy', allow_pickle=True)
    params = params[()]

    np.random.seed(0)
    tf.random.set_seed(0)
    model = simple_deep_esn_classifier.SimpleDeepESNClassifier(num_classes=NUM_CLASSES, units=UNITS, layers=LAYERS,
                                                               return_sequences=SEQ_TO_SEQ, concat=CONCAT, leaky=LEAKY,
                                                               spectral_radius=SPECTRAL_RADIUS,
                                                               input_scaling=INPUT_SCALING)
    model.build(input_shape=(1, None, 3))
    model.set_w(params)

    rospy.init_node('listener', anonymous=True)

    reads = {'EDA': [], 'HR': []}
    tmp_reads = {'EDA': [], 'HR': []}

    PPGtoHR = PPGtoHRAlgorithm(
        sampling_rate=64, number_of_beats_to_average=1, use_last_estimate=1)

    # Publisher and subscriber
    stress_pub = rospy.Publisher('stress_publisher', Float32, queue_size=10)
    # rospy.Subscriber("shimmer_imu_pub", Float32MultiArray, callback, (model,reads,tmp_reads, stress_pub), queue_size= 10)
    rospy.Subscriber("shimmer_imu_pub", Float32MultiArray, callback, (model,
                     reads, tmp_reads, PPGtoHR, stress_pub), queue_size=10)  # if ppgtohr used

    rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("Python node has been started!")
    setup()
