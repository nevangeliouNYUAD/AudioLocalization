#!/usr/bin/env python
import rospy
import time
import sys
import cv2
import struct
from scipy.io import loadmat, wavfile
import wave
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import pyaudio
import queue


from utils import *

buff = []
fulldata = []
audio_data = Float32MultiArray()
RATE = 48000
FRAMES_PER_BUFFER = 1000
CHANNELS = 1
MSG_DURATION = 1.0 #sec

#wf = wave.open('/home/risc/work/projects/Drone_surviliance_system/data_collection/data/1102/drone1_exp_20221102-110845.wav', 'rb')

def callback(in_data, frame_count, time_info, flag):
    global fulldata #global variables for filter coefficients and array
    #in_data = struct.unpack( "f"*frame_count*CHANNELS, in_data )
    #in_data = wf.readframes(frame_count)
    buff.append(in_data)

    fulldata.append(in_data) #saves filtered data in an array
    return (None, pyaudio.paContinue)

def start_node(stream, waveFile, pub, frames_per_buffer=96000, channels=16, sr=96000):
    #rospy.init_node('image_pub')
    rospy.loginfo('audio record node started')
    
    while not rospy.is_shutdown():
        #print(len(buff))
        if(len(buff)>(RATE/FRAMES_PER_BUFFER)*MSG_DURATION):
            data = buff[:48]
            del buff[:48]

            data = b''.join(data)
            in_data = np.frombuffer(data, dtype='<i2').astype(np.float32)
            audio_data.data = in_data
            rospy.loginfo("read audio")
            print(in_data.shape)
            pub.publish(audio_data)
            #in_data = in_data.astype(np.float32)
            #spectrum = audio2spectrum(in_data, sr, channels)
            #print(spectrum.shape)
            #spectrum = cv2.resize(spectrum, (320, 160))
            #print(spectrum.shape)
            #cv2.imshow('frame', spectrum)
            #k = cv2.waitKey(30) & 0xff 
            #if k == 27: 
            #    break
            #cv2.destroyAllWindows() 
            # # if ret is not None:
            # #imgMsg = bridge.cv2_to_imgmsg(spectrum[:,:,:3], "bgr8")
            # audio_data.data = in_data
                
            # rospy.loginfo("read audio")
            # pub.publish(audio_data)
        rospy.Rate(100.0).sleep()

    waveFile.writeframes(b''.join(fulldata))
    stream.stop_stream()
    waveFile.close()
    print("Shutting down")


def main():
    rospy.init_node('audio_pub', anonymous=True)
    bridge = CvBridge()
    format = pyaudio.paInt16
    audio = pyaudio.PyAudio()
    
    stream = audio.open(format=format,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer = FRAMES_PER_BUFFER,
                        stream_callback=callback)
    stream.start_stream()
    # play_stream = audio.open(format=format,
    #                     channels=channels,
    #                     rate=rate,
    #                     output=True,
    #                     frames_per_buffer = frames_per_buffer)
            
    # audio_frames = []
    # start = time.time()
    # data = stream.read(frames_per_buffer)
    # audio_frames.append(data)
    #print(data)
    #print(time.time() - start)
    
    # waveFile = wave.open('./test.wav', 'wb')
    # waveFile.setnchannels(channels)
    # waveFile.setsampwidth(audio.get_sample_size(format))
    # waveFile.setframerate(rate)
    # waveFile.writeframes(b''.join(audio_frames))
    # waveFile.close()

    # data = np.frombuffer(data, dtype='<i2').reshape(-1, channels)
    # in_data = data.astype(np.float32)
    # spectrum = audio2spectrum(in_data, rate, channels)
    # print(spectrum.shape, data.shape)
    # print(data, data.max(), data.min(), data.shape)

    # sr, x = wavfile.read('./test.wav')
    # print(x, x.max(), x.min(), x.shape)
    #exit()

    #video_path = '/data/detection/Spherical_Detection_Evangeliou/videos/video1/result_equirectangular.avi'
    #cap = cv2.VideoCapture(2)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    audio_filename = "/home/risc5/projects/mics_track/log/{0}_{1}.wav".format("exp", timestr)
    waveFile = wave.open(audio_filename, 'wb')
    waveFile.setnchannels(CHANNELS)
    waveFile.setsampwidth(audio.get_sample_size(format))
    waveFile.setframerate(RATE)

    pub = rospy.Publisher('/audio/spectrum',Float32MultiArray, queue_size=3)
    #pub = rospy.Publisher('/audio/spectrum', Image, queue_size=5)
    start_node(stream, waveFile, pub)
    try:
        rospy.spin()
    except KeyboardInterrupt:    
        print("Shutting down")

if __name__ == '__main__':
    main()
