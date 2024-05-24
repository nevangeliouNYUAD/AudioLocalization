#!/usr/bin/env python
import os,sys
import rospy
import torch
import numpy as np
import torch.backends.cudnn as cudnn
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Float32MultiArray

import torch.nn.functional as F
import torch.distributed as dist
from timm.utils import AverageMeter

import matplotlib.pyplot as plt

from swin import SwinTransformer
from utils import *
from sensor_msgs.msg import Image

sr = 48000
channels = 1
pan_tilt_angles = Float32MultiArray()
bridge = CvBridge()

def detect_audio(signal):
    in_data = np.array(signal.data)
    spectrum = audio2spectrum(in_data, sr, channels)
    spectrum = cv2.resize(spectrum, (640, 320))

    librosa.display.specshow(librosa.power_to_db(spectrum,                                              
        ref=np.max), y_axis='mel', fmax=1000, x_axis='time')
    plt.colorbar(format='%+2.0f dB')
    plt.title('Mel spectrogram')
    plt.tight_layout()
    plt.pause(0.0001)
    plt.clf()
    rospy.loginfo("showed spectrum")

if __name__ == "__main__":
    rospy.init_node('audio_show', anonymous=True)
    rospy.Subscriber("/audio/spectrum", Float32MultiArray, detect_audio)
    rospy.spin()    
    