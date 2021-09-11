import sys
import os
import rospy
import datetime
from time import strftime
from time import gmtime
import queue
import time
import math
from rospy.client import spin
from rospy.core import rospyinfo
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from scipy.signal import filtfilt, butter
from sklearn import metrics
import csv
import pickle
import warnings
import neurokit2 as nk
#import tensorflow as tf
from models.ppg_to_hr import PPGtoHRAlgorithm
# from src.models import simple_deep_esn_classifier # for roscore
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import * 
from PyQt5.QtGui import * 
from PyQt5.QtCore import Qt, QSize    
from enum import IntEnum

DIR = "/home/furkan/Workspaces/my_example/src/my_example_pkg/src/participants_parameters/"
#change the subject name and experiment type during the data recording
SUBJECT_NAME = "subject_9"
EXPERIMENT_TYPE = "_with_shared_control" 
#EXPERIMENT_TYPE = "_without_shared_control" 
Title = ["System_time","game_time", "current_score", "mean_hr", "current_hr", "mean_scl", "mean_scr", "is_stressed", "Event_type"]
Event_types = ['GAME_NOT_STARTED','COLLIDE_WITH_WALL', 'COLLIDE_WITH_BOX', 'ROBOT_CONTROL', 'SHARED_ROBOT_CONTROL']
current_event = 'GAME_NOT_STARTED'
save_before_change = False
# combine the file directory, name and type
FILE_DIR_AND_NAME = DIR + SUBJECT_NAME + EXPERIMENT_TYPE + '.csv'

SHARED_CONTROL_MECHANISM = True
if EXPERIMENT_TYPE == "_with_shared_control":
    SHARED_CONTROL_MECHANISM = True # Change by type of experiment
elif EXPERIMENT_TYPE == "_without_shared_control":
    SHARED_CONTROL_MECHANISM = False # Change by type of experiment
else:
    assert 0
    
SAMPLE_RATE = 48.0 #Hz
RESAMPLE_RATE = 8.0 #Hz
WINDOW_LIMIT = 120 #initial baseline duration (2 sec), it is updated after collecting the baseline values
PREDICTION_FREQUENCY = 1  # second
STRESS_THRESHOLD = 0.5
MODEL_BASELINE_MEAN = 62.8360 #S5 baseline this value is for calibration 

#Stress declerations
current_score = 100
game_elapsed_time = 300
is_stressed = False
prev_score = 100
score_color_change_time = 0.0


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

def update_events(event_indx):
    global current_event
    global save_before_change
    if not save_before_change:
        current_event = Event_types[int(event_indx)]

    if current_event == 'COLLIDE_WITH_WALL' or current_event == 'COLLIDE_WITH_BOX':
        save_before_change = True


class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self, None, QtCore.Qt.WindowStaysOnTopHint)

        # set the title
        self.setWindowTitle("INFORMATION BOARD")
  
        # setting  the geometry of window
        self.setGeometry(0, 0, 420, 400)
  
        # creating a label widget
        self.label_1 = QLabel("Score:", self)
        self.label_2 = QLabel("Time:", self)
        self.label_3 = QLabel("Stress Status:", self)

        # For background color
        self.label1_palet = QPalette(self.label_1.palette())
        self.label1_palet.setColor(QPalette.Background, QColor(Qt.green))
        self.label_1.setPalette(self.label1_palet)
        self.label_1.setAutoFillBackground(True)

        self.label2_palet = QPalette(self.label_2.palette())
        self.label2_palet.setColor(QPalette.Background, QColor(Qt.green))
        self.label_2.setPalette(self.label2_palet)
        self.label_2.setAutoFillBackground(True)

        self.label3_palet = QPalette(self.label_3.palette())
        self.label3_palet.setColor(QPalette.Background, QColor(Qt.green))
        self.label_3.setPalette(self.label3_palet)
        self.label_3.setAutoFillBackground(True)

        # self.label_1.setFont(QFont('Times', 13))
        # self.label_2.setFont(QFont('Times', 13))
        # self.label_3.setFont(QFont('Times', 13))
        self.label_1.setStyleSheet("font: 15pt; font-family: Calibri")
        self.label_2.setStyleSheet("font: 15pt; font-family: Calibri")
        self.label_3.setStyleSheet("font: 15pt; font-family: Calibri")

        self.label_1.setGeometry(10, 50,  400, 75)
        self.label_2.setGeometry(10, 150, 400, 75)
        self.label_3.setGeometry(10, 250, 400, 125)

    def callback(self, msg):
        global SHARED_CONTROL_MECHANISM
        global is_stressed
        global prev_score
        global score_color_change_time
        global current_score
        global game_elapsed_time
        data = msg.data
        # changing the text of label
        current_score = data[0]
        game_elapsed_time = data[1]
        is_succesfull = data[2]
        inform_user_about_control = data[3]
        update_events(data[4])
        if is_succesfull:
            self.label_1.setText("Congratulations ! You did it!\n You got 100 score point!")
            current_score = 100
            self.label_2.setText("Time: " + str(datetime.timedelta(seconds=float(format(game_elapsed_time, '.2f')))))
        else:
            if current_score > 0 :
                if prev_score != current_score:
                    score_color_change_time = time.time()
                
                if time.time() - score_color_change_time < 1:
                    self.label1_palet.setColor(QPalette.Background, QColor(Qt.red))
                else:
                    self.label1_palet.setColor(QPalette.Background, QColor(Qt.green))
                    
                self.label_1.setPalette(self.label1_palet)
                self.label_1.setText("Score: " + str(data[0]))
                prev_score = current_score

            else:
                self.label_1.setText("Score: 0 \n GAME OVER")


            if(game_elapsed_time > 0 ):
                if game_elapsed_time > 30:
                    self.label2_palet.setColor(QPalette.Background, QColor(Qt.green))
                else:
                    if int(game_elapsed_time) % 2 == 0:
                        self.label2_palet.setColor(QPalette.Background, QColor(Qt.red))
                    else:
                        self.label2_palet.setColor(QPalette.Background, QColor(Qt.green))
                    
                self.label_2.setPalette(self.label2_palet)
                self.label_2.setText("Time: " + str(datetime.timedelta(seconds=float(format(game_elapsed_time, '.2f')))))

            else :
                self.label_2.setText("TIME IS UP ! GAME OVER")
            
            if SHARED_CONTROL_MECHANISM:
                if not is_stressed:
                    if not inform_user_about_control:
                        self.label_3.setText("Stress Status: Not stressed")
                        self.label3_palet.setColor(QPalette.Background, QColor(Qt.green))
                    else:
                        self.label_3.setText("Stress Status: You are relaxing... :) \n\nBe Ready! The control will be given to \nyou again in 5 seconds.")
                        self.label3_palet.setColor(QPalette.Background, QColor(Qt.magenta))
                else :
                    self.label_3.setText("Stress Status: You are stressful!\n\nI'm helping you now, please try to relax")
                    self.label3_palet.setColor(QPalette.Background, QColor(Qt.yellow))

            self.label_3.setPalette(self.label3_palet)

        # show all the widgets
        self.show()

    def display_calibration_time(self, displayed_text):
        self.label_3.setText(displayed_text)
        # show all the widgets
        self.show()



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


def write_to_csv_file(data_to_be_saved):
    global FILE_DIR_AND_NAME
    with open(FILE_DIR_AND_NAME, 'a') as f:
            writer = csv.writer(f)
            writer.writerow(data_to_be_saved)

baseline_difference_hr = 0.0
baseline_difference_scl = 0.0

def signal_process(data, model, pub, system_time):

    # timestamps = []
    global is_stressed
    global save_before_change
    global baseline_difference_hr
    global baseline_difference_scl

    hr = []
    eda = []

    # Remember that EDA and HR are at the same frequency, so here they have the same length
    for i in range(len(data['HR'])):
        hr.append(data['HR'][i][0])
        eda.append(data['EDA'][i][0])
    try:
        eda = butter_lowpass_filter(eda, 1, SAMPLE_RATE, order=4)
        # Applying NeuroKit2 on EDA
        signal, process = nk.eda_process(eda, sampling_rate=SAMPLE_RATE)
        scr = signal['EDA_Phasic'].to_numpy() #EDA phasic is filtered EDA signal
        scr = scr.reshape((scr.shape[0], 1))
        scl = signal['EDA_Tonic'].to_numpy()  #EDA Tonic is EDA signal 
        scl = scl.reshape((scl.shape[0], 1))
        # print("diff: {}  scr: {}   scl: {}".format(baseline_difference_hr, np.mean(scr), np.mean(scl)))
        
        # Now we have to resample the signals to 20 Hz
        current_hr = hr[-1]
        # hr = downsampling_mean(hr, SAMPLE_RATE, RESAMPLE_RATE)
        # scr = downsampling_mean(scr, SAMPLE_RATE, RESAMPLE_RATE)
        # scl = downsampling_mean(scl, SAMPLE_RATE, RESAMPLE_RATE)

        mean_hr  = np.mean(hr) + baseline_difference_hr
        mean_scl = np.mean(scl) - baseline_difference_scl
        mean_scr = np.mean(scr)
        # HR - Eda Tonic - Eda phasic 

        inp = np.array([mean_hr, mean_scl, mean_scr])
        y = model.predict(inp.reshape(1,-1))
        is_stressed = metrics.accuracy_score([2], y)
        #if equal 2 -> stressful condition if equal -> 1 not stressful
        print("calibrated mean_hr: {} current_hr: {} tonic: {} phasic: {} Sressed: {}".format(mean_hr, current_hr, mean_scl, mean_scr, is_stressed))   
        
        data_to_be_saved = [system_time, datetime.timedelta(seconds=float(format(game_elapsed_time, '.2f'))), current_score, mean_hr, 
                                                             current_hr, mean_scl, mean_scr, is_stressed, current_event]
        save_before_change = False
        
        # Save all the collected physiological parameters to the csv file
        write_to_csv_file(data_to_be_saved)

        stressed_information_pack = Float32MultiArray()
        
        stressed_information_pack.data.append(is_stressed)
        stressed_information_pack.data.append(SHARED_CONTROL_MECHANISM)
        pub.publish(stressed_information_pack)  # Ros - publish whether the person get stressed or not

    except ValueError:
        pass
    
    

is_first_min = True
is_sec_passed = False
calculated_mean_hr = 0.0
start_time = time.time()
baseline_time = []
static_time_stamp = ''

def callback(msg, args):
    global is_first_min
    global calculated_mean_hr
    global baseline_difference_hr
    global baseline_difference_scl
    global start_time
    global is_sec_passed
    global baseline_time
    global static_time_stamp
    global WINDOW_LIMIT

    packet = msg.data
    model = args[0] #Machine learning model
    reads = args[1]
    tmp_reads = args[2]
    PPG2HR = args[3]  # if ppgtohr used
    publisher = args[4] #Ros node publisher
    displayer = args[5]

    # Collect the data for every second
    if not is_sec_passed:
        elapsed_time = time.time() - start_time
        synced_timestamp = time_syncer(packet[Shimmer_pack.time_stamp])[2]
        timestamp = datetime.datetime.fromtimestamp(synced_timestamp).strftime("%H:%M:%S.%f")[:-3]  # normal time format
        ppg = packet[Shimmer_pack.PPG_RAW]
        hr = PPG2HR.ppg_to_hr(ppg, synced_timestamp * 1000)[0]
        if elapsed_time > 15: # Wait 10 second after node has started
            is_sec_passed = True
            print("Participant's baseline datas are being collected...")
            baseline_wait_time = strftime("%M:%S", gmtime(WINDOW_LIMIT)) + " min"
            displayer.display_calibration_time("Parameters are being collected...\n\nPlease be relax and wait for " + baseline_wait_time)
            
    
    readden = len(tmp_reads['EDA'])    
    if readden < int(PREDICTION_FREQUENCY * SAMPLE_RATE) and is_sec_passed:
        # Synchronizing timestamp
        readden += 1  # total received packet bytes size
        # synced_timestamp = packet[Shimmer_pack.time_stamp]
        synced_timestamp = time_syncer(packet[Shimmer_pack.time_stamp])[2]
        timestamp = datetime.datetime.fromtimestamp(synced_timestamp).strftime("%H:%M:%S.%f")[:-3]  # normal time format
        
        
        static_time_stamp = timestamp
        # print("synced_timestamp:{}    packet[2]:{}".format(synced_timestamp, packet[2]))

        # We use the calculated heart-rate by C++ code, or we can also calculate by using python script
        ppg = packet[Shimmer_pack.PPG_RAW]
        hr = PPG2HR.ppg_to_hr(ppg, synced_timestamp * 1000)[0]

        # ppg = packet[Shimmer_pack.PPG_CALIBRATED]
        # hr = packet[Shimmer_pack.heart_rate] # use calculated heart-rate by C++ code

        eda = packet[Shimmer_pack.GSR_SKIN_CONDUCTANCE]
        
        # print("HR:{}    EDA:{}    TimeStamp:{}".format(hr, eda, timestamp))
        tmp_reads['HR'].append([hr])
        tmp_reads['EDA'].append([eda])
      
    else:
        # We want the last 30 second data (WINDOW_LIMIT = 30 and 64 is the sample rate)
        # Here we get rid of the first second data if the are more than 30 seconds of data
        how_much = int(PREDICTION_FREQUENCY * SAMPLE_RATE)
        to_append_HR = tmp_reads['HR'][0:how_much]
        to_append_EDA = tmp_reads['EDA'][0:how_much]
        tmp_reads['HR'] = tmp_reads['HR'][how_much:]
        tmp_reads['EDA'] = tmp_reads['EDA'][how_much:]

        if is_first_min:
            baseline_time.append(static_time_stamp)
            if len(tmp_reads['HR']) > 0:
                print(tmp_reads['HR'][-1])

        if len(reads['HR']) >= int(WINDOW_LIMIT * SAMPLE_RATE) and len(reads['EDA']) >= int(WINDOW_LIMIT * SAMPLE_RATE) :
            # clear all 30sec data frame

            if is_first_min:
                #Execute for one time before implement the stress detection algorithm (Baseline Calculations)
                is_first_min = False
                mean_baseline_hr = np.mean(reads['HR'])
                baseline_difference_hr = MODEL_BASELINE_MEAN - mean_baseline_hr
                print("Participant's mean Heart-rate: {}    HR diff: {}".format(mean_baseline_hr, baseline_difference_hr))
                displayer.display_calibration_time("Are you ready to begin?")
                # open a csv file to save the participants parameters during the experiment
                write_to_csv_file(Title)
                
                base_eda = []
                base_hr  = []
                
                for i in range(len(reads['EDA'])):
                    base_eda.append(reads['EDA'][i][0])
                    base_hr.append(reads['HR'][i][0])
               
                base_eda = butter_lowpass_filter(base_eda, 1, SAMPLE_RATE, order=4)
                # Applying NeuroKit2 on EDA
                signal, process = nk.eda_process(base_eda, sampling_rate=SAMPLE_RATE)
                scr = signal['EDA_Phasic'].to_numpy() #EDA phasic is filtered EDA signal
                scr = scr.reshape((scr.shape[0], 1))
                scl = signal['EDA_Tonic'].to_numpy()  #EDA Tonic is EDA signal 
                scl = scl.reshape((scl.shape[0], 1))
                
                mean_baseline_scl = np.mean(scl)
                mean_baseline_scr = np.mean(scr)
                baseline_difference_scl = mean_baseline_scl
                
                baseline_time = baseline_time[len(baseline_time)-WINDOW_LIMIT:]
                for indx in range(WINDOW_LIMIT):
                    mean_base_hr = np.mean(base_hr[0:how_much])
                    mean_base_scl = np.mean(scl[0:how_much])
                    mean_base_scr = np.mean(scr[0:how_much])

                    base_hr = base_hr[how_much:]
                    scl = scl[how_much:]
                    scr = scr[how_much:]
                    
                    baseline_to_csv = [baseline_time[indx], 'N/A','N/A','N/A', mean_base_hr, mean_base_scl, mean_base_scr, 'N/A', 'BASELINE_DATA']
                    write_to_csv_file(baseline_to_csv)

                write_to_csv_file([static_time_stamp, '***','***', mean_baseline_hr, '***', mean_baseline_scl, mean_baseline_scr, '***', 'BASELINE_MEAN_VALUES'])
                
                WINDOW_LIMIT = 30
                len_from = len(reads['HR']) - int(WINDOW_LIMIT * SAMPLE_RATE)
                reads['HR'] = reads['HR'][len_from:]
                reads['EDA'] = reads['EDA'][len_from:]

            reads['HR'] = reads['HR'][int(PREDICTION_FREQUENCY * SAMPLE_RATE):]
            reads['EDA'] = reads['EDA'][int(PREDICTION_FREQUENCY * SAMPLE_RATE):]

        reads['HR'] = reads['HR'] + to_append_HR
        reads['EDA'] = reads['EDA'] + to_append_EDA            
        
        # Get the first 64 packets from reads
        if not is_first_min:
            signal_process(reads, model, publisher, static_time_stamp)            


def setup():
    # load ML model
    #os.makedirs(os.path.dirname('/home/furkan/Workspaces/my_example/knn_model.pickle'), exist_ok=True)
    file_path = '/home/furkan/Workspaces/my_example/src/my_example_pkg/src/trained_ML_model/svm_model.pickle'
    with open(file_path, 'rb') as f:
        get_model = pickle.load(f)

    # check whether participants file already exist
    if os.path.exists(FILE_DIR_AND_NAME):
        print("You are trying to create a file name that already exists!")
        assert 0
    else:
        print("file does not exist... OK!")

    #Score board
    app = QtWidgets.QApplication(sys.argv)
    mainWin = MainWindow()

    rospy.init_node('Ml_model', anonymous=True)

    reads = {'EDA': [], 'HR': []}
    tmp_reads = {'EDA': [], 'HR': []}

    PPGtoHR = PPGtoHRAlgorithm(sampling_rate=SAMPLE_RATE, number_of_beats_to_average=1, use_last_estimate=1)

    # Publisher and subscriber
    stress_pub = rospy.Publisher('stress_publisher', Float32MultiArray, queue_size=10)
    # rospy.Subscriber("shimmer_imu_pub", Float32MultiArray, callback, (model,reads,tmp_reads, stress_pub), queue_size= 10)
    rospy.Subscriber("shimmer_imu_pub", Float32MultiArray, callback, (get_model,
                     reads, tmp_reads, PPGtoHR, stress_pub, mainWin), queue_size=10)  # if ppgtohr used
    
    rospy.Subscriber("score_and_time", Float32MultiArray, mainWin.callback, queue_size=10) 

    mainWin.show()
    sys.exit( app.exec_())
    rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("ML node has been started!")
    
    setup() #set up the system and wait up coming dataset
    
    