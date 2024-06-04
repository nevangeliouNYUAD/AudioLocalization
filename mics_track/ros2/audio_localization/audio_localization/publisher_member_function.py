import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray
import numpy as np
import pyaudio
import wave
import time

import struct
from scipy.io import loadmat, wavfile


buff = []
fulldata = []
audio_data = Float32MultiArray()
RATE = 48000
FRAMES_PER_BUFFER = 1000
CHANNELS = 1
MSG_DURATION = 1.0 #sec

def callback(in_data, frame_count, time_info, flag):
    global fulldata #global variables for filter coefficients and array
    #in_data = struct.unpack( "f"*frame_count*CHANNELS, in_data )
    #in_data = wf.readframes(frame_count)
    buff.append(in_data)

    fulldata.append(in_data) #saves filtered data in an array
    return (None, pyaudio.paContinue)

class AudioPublisher(Node):

    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/audio/spectrum', 10)
        timer_period = 0.033  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
         if(len(buff)>(RATE/FRAMES_PER_BUFFER)*MSG_DURATION):
            data = buff[:48]
            del buff[:48]

            data = b''.join(data)
            in_data = np.frombuffer(data, dtype='<i2').astype(np.float32)
            print(in_data.shape)
            audio_data.data = in_data.tolist()
            print(audio_data.data[0])
            self.get_logger().info("read audio")
            self.publisher_.publish(audio_data)
            #self.get_logger().info('Publishing')
            #self.i += 1

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    audio_publisher = AudioPublisher()

    format = pyaudio.paInt16
    audio = pyaudio.PyAudio()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    audio_filename = "/home/nikolaos/projects/audio_localization/logs/{0}_{1}.wav".format("exp", timestr)
    waveFile = wave.open(audio_filename, 'wb')
    waveFile.setnchannels(CHANNELS)
    waveFile.setsampwidth(audio.get_sample_size(format))
    waveFile.setframerate(RATE)
    
    stream = audio.open(format=format,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer = FRAMES_PER_BUFFER,
                        stream_callback=callback)
    stream.start_stream()
    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        print("Shutting down")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waveFile.writeframes(b''.join(fulldata))
    stream.stop_stream()
    waveFile.close()
    audio_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
