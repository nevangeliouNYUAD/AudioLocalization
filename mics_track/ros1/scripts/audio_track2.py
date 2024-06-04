#!/usr/bin/env python
import os,sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray
from timm.utils import AverageMeter
import matplotlib.pyplot as plt
from utils import *
from sensor_msgs.msg import Image

buff = []
yaw_commands = []
overall_rms_values = []
yaw_angle = Float32MultiArray()
current_range = [0,360.0]
RANGE_REDUCE_FACTOR = 0.25
NUM_BUFFERS = 5

# 0 alarm
# 1 TV
# 2 toilet
# .....

# ros message: Float Array [1 , [48000 audio samples]]

def detect_audio(signal):
    #Conversion from ros Float32 Multiaarray to numpy array
    in_data = np.array(signal.data)
    #Create a 1 second buffer and compare sound intensity
    #Buff is a list of NDArrays, each consisting of 10240 samples a.k.a. 0.213msec
    buff.append(in_data)
    
    #spectrum = audio2spectrum(in_data, sr, channels)
    #spectrum = cv2.resize(spectrum, (640, 320))
    #librosa.display.specshow(librosa.power_to_db(spectrum,                                              
    #    ref=np.max), y_axis='mel', fmax=1000, x_axis='time')
    #plt.colorbar(format='%+2.0f dB')
    #plt.title('Mel spectrogram')
    #plt.tight_layout()
    #plt.pause(0.0001)
    #plt.clf()
    #rospy.loginfo("audio tracked spectrum")

def audio_classification_cb(class_):
    if(len(buff)>NUM_BUFFERS):
        data = buff[:NUM_BUFFERS] #always equal to 5 seconds
        del buff[0:NUM_BUFFERS]

        # Compute the square of each element in every array
        squared_arrays = [np.square(array) for array in data]

        # Compute the mean of the squared elements in each array
        mean_squared_arrays = [np.mean(array) for array in squared_arrays]

        # Compute the square root of the mean squared elements to get the RMS value
        rms_values = [np.sqrt(mean_squared_array) for mean_squared_array in mean_squared_arrays]

        # Compute the rms value for 5 sec 
        overall_rms_values.append(np.mean(rms_values))

    #Based on how many values you have, move dynamixel to the desired location
    if(len(overall_rms_values) < len(yaw_commands)):
        if( (len(yaw_angle.data)>0) and (yaw_angle.data[0] != yaw_commands[len(overall_rms_values)])):
            yaw_angle.data.clear()
            yaw_angle.data.append(yaw_commands[len(overall_rms_values)])
            pub_yaw.publish(yaw_angle)
            #Dynamixel makes sound
            rospy.Rate(3.0).sleep()
            buff.clear()
    else:
        elements = overall_rms_values[0:len(yaw_commands)]
        print("RMS values")
        print(elements)
        max_value = max(elements)
        max_index = elements.index(max_value)
        go_to = yaw_commands[max_index]
        print("Going to: ", go_to)
        yaw_angle.data.clear()
        yaw_angle.data.append(go_to)
        pub_yaw.publish(yaw_angle)
        #Dynamixel makes sound
        rospy.Rate(3.0).sleep()
        buff.clear()
        overall_rms_values.clear()
        print("Reschuffling go to values")
        yaw_commands.clear()
        previous_range = current_range[1] - current_range[0]
        current_range.clear()
        current_range.append(go_to - previous_range/4.0)
        current_range.append(go_to + previous_range/4.0)

        yaw_commands.append(current_range[0])
        yaw_commands.append(go_to - (current_range[1] - current_range[0])/4.0)
        yaw_commands.append(go_to)
        yaw_commands.append(go_to + (current_range[1] - current_range[0])/4.0)
        if current_range[1] % 360.0 != 0.0:
            yaw_commands.append(current_range[1])

        #current_range.append(go_to + previous_range/4.0)
        #current_range.append(go_to + previous_range/8.0)
        #current_range.append(go_to - previous_range/8.0)
        #current_range.append(go_to - previous_range/4.0)
        print(yaw_commands)
        print("RESTART!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

if __name__ == "__main__":
    rospy.init_node('audio_track', anonymous=True)
    pub_heat_map = rospy.Publisher('/camera/rgb/heat_map', Image, queue_size=10)
    pub_yaw = rospy.Publisher('/audio/yaw_angle', Float32MultiArray, queue_size=10)
               
    rospy.Subscriber("/audio/spectrum", Float32MultiArray, detect_audio)
    rospy.Subscriber("/audio/classification", Float32MultiArray, audio_classification_cb)
    median_value = current_range[0] + (current_range[1] - current_range[0])/2.0
    yaw_commands.append(current_range[0])
    yaw_commands.append(median_value - (current_range[1] - current_range[0])/4.0)
    yaw_commands.append(median_value)
    yaw_commands.append(median_value + (current_range[1] - current_range[0])/4.0)
    if current_range[1] % 360.0 != 0.0:
        yaw_commands.append(current_range[1])
    yaw_angle.data.append(yaw_commands[0])
    pub_yaw.publish(yaw_angle)
    #Dynamixel makes sound
    rospy.Rate(3.0).sleep()
    yaw_angle.data.clear()
    buff.clear()
    rospy.spin() 
