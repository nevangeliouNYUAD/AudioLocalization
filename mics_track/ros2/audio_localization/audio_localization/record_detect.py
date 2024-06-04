import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray
import numpy as np
import pyaudio
import wave
import time
from dynamixel_sdk import *
from threading import Thread

import struct
from scipy.io import loadmat, wavfile


#Spectrum is saved here
buff = []
#Full recording -- excludes Dynamixel motion
fulldata = []
#Variable to store the publishing data
audio_data = Float32MultiArray()
#Audio card sampling rate
RATE = 48000
#How long do you want your message to be in seconds
MSG_DURATION = 5.0
#No of samples that indicate to PyAudio one buffer is filled. The assigned callback is run from here.
FRAMES_PER_BUFFER = int (RATE * MSG_DURATION)
#Number of channels from the microphone
CHANNELS = 1
#List of yaw commands to execute to find heading
yaw_commands = []
#RMS value for previous variable. Size is equal to yaw_commands
overall_rms_values = []
#When 5 buffers are filled with data, we will calculate one RMS value and store it in *overall_rms_values*
NUM_BUFFERS = 5

MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRO_PRESENT_VELOCITY   = 128
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4096         # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600
dxl_goal_position           = 0 #Where to go
dxl_moving                  = False

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 5    # Dynamixel moving status threshold in degress

def callback(in_data, frame_count, time_info, flag):
    global fulldata #global variables for filter coefficients and array
    #in_data = struct.unpack( "f"*frame_count*CHANNELS, in_data )
    #in_data = wf.readframes(frame_count)
    buff.append(in_data)

    #fulldata.append(in_data) #saves filtered data in an array
    return (None, pyaudio.paContinue)

class AudioSubscriber(Node):
    def __init__(self):
        global dxl_moving
        global dxl_goal_position
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/audio/spectrum',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

        # Apply configuration
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID,  108, 1000)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID,  112, 1000)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID,  84, 200) #P Gain
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID,  48, DXL_MAXIMUM_POSITION_VALUE) #MAX_POS_LIMIT
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID,  52, DXL_MINIMUM_POSITION_VALUE) #MIN_POS_LIMIT

        # Get angle
        self.dxl_current_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        print("current position yaw degrees", int(self.dxl_current_position*180.0/2048))

        # Set first angle
        dxl_goal_position = 0
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_POSITION, 0)
        dxl_moving = True
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        global dxl_goal_position
        global dxl_moving
        #print("dxl goal position", dxl_goal_position)
        #print("dxl current position", self.dxl_current_position)
        #first check of there is a goal position
        self.dxl_current_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        self.dxl_current_position = int(self.dxl_current_position * 180 / 2048)
        if(abs(dxl_goal_position-self.dxl_current_position)>DXL_MOVING_STATUS_THRESHOLD):
            #print("dxl new goal position", dxl_goal_position)
            #print("dxl current position", self.dxl_current_position)
            #Module normalizes between -360 to 360
            dxl_goal_position = dxl_goal_position % 360
            #Negative is positive
            if dxl_goal_position < 0:
                dxl_goal_position += 360
            #print("dxl goal position", dxl_goal_position)
            dxl_moving = True
            self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_GOAL_POSITION, int(dxl_goal_position*2048/180))
        else:
            dxl_moving = False

    def listener_callback(self, msg):
        global dxl_goal_position
        global overall_rms_values
        global yaw_commands
        data = msg.data #always equal to 5 seconds

        # Compute the square of each element in every array
        squared_arrays = [np.square(array) for array in data]

        # Compute the mean of the squared elements in each array
        mean_squared_arrays = [np.mean(array) for array in squared_arrays]

        # Compute the square root of the mean squared elements to get the RMS value
        rms_values = [np.sqrt(mean_squared_array) for mean_squared_array in mean_squared_arrays]

        # Compute the rms value for 5 sec 
        overall_rms_values.append(10 * np.mean(rms_values))
        
        #Based on how many values you have, move dynamixel to the desired location
        if(len(overall_rms_values) < len(yaw_commands)):
            #if( (len(yaw_angle.data)>0) and (yaw_angle.data[0] != yaw_commands[len(overall_rms_values)])):
            dxl_goal_position = yaw_commands[len(overall_rms_values)]
        else:
            elements = overall_rms_values[0:len(yaw_commands)]
            print("RMS values")
            print(elements)
            max_value = max(elements)
            max_index = elements.index(max_value)
            #Dynamixel makes sound
            overall_rms_values.clear()
            print("Reschuffling go to values")
            if(len(yaw_commands)>3):
                dxl_goal_position = yaw_commands[max_index]
                print("Going to: ", dxl_goal_position)
                yaw_commands.clear()
                yaw_commands.append(dxl_goal_position)
                yaw_commands.append(dxl_goal_position - 45)
                yaw_commands.append(dxl_goal_position + 45)
            else:
                dxl_goal_position = 0
                print("Going to: ", dxl_goal_position)
                yaw_commands.clear()
                yaw_commands.append(0)
                yaw_commands.append(90)
                yaw_commands.append(180)
                yaw_commands.append(270)
            print(yaw_commands)
            print("RESTART!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


class AudioPublisher(Node):

    def __init__(self):
        super().__init__('audio_publisher')
        self.timer_period = 0.1  # seconds
        self.publisher_ = self.create_publisher(Float32MultiArray, '/audio/spectrum', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.delete_next_buffer = True

    def timer_callback(self):
         global dxl_moving
         if dxl_moving == True:
            self.delete_next_buffer = True
         if(len(buff)>=1):
            if self.delete_next_buffer == False:
                data = buff[:1]
                del buff[:1]
                data = b''.join(data)
                in_data = np.frombuffer(data, dtype='<i2').astype(np.float32)
                audio_data.data = in_data.tolist()
                self.get_logger().info("Got audio")
                self.publisher_.publish(audio_data)
                if dxl_moving == False:
                    #save data
                    fulldata.append(data) #saves filtered data in an array
            else:
                self.delete_next_buffer = False
                del buff[:1]

def spin_node(node):
    rclpy.spin(node)

def main(args=None):
    global dxl_goal_position
    global yaw_commands
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    audio_publisher = AudioPublisher()
    audio_subscriber = AudioSubscriber()

    audio_executor = MultiThreadedExecutor()
    audio_sub_executor = MultiThreadedExecutor()

    audio_executor.add_node(audio_publisher)
    audio_sub_executor.add_node(audio_subscriber)

    audio_publisher_thread = Thread(target=audio_executor.spin)
    audio_subscriber_thread = Thread(target=audio_sub_executor.spin)

    ##############################   AUDIO   ################################################

    format = pyaudio.paInt16
    audio = pyaudio.PyAudio()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    audio_filename = "/home/nikolaos/projects/audio_localization/logs/tests/{0}_{1}.wav".format("exp", timestr)
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

    yaw_commands.append(0)
    yaw_commands.append(90)
    yaw_commands.append(180)
    yaw_commands.append(270)

    ##############################   ROS   ################################################

    #LOOP
    audio_publisher_thread.start()
    audio_subscriber_thread.start()
    
    try:
        audio_publisher_thread.join()
        audio_subscriber_thread.join()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waveFile.writeframes(b''.join(fulldata))
    stream.stop_stream()
    waveFile.close()
    audio_publisher.destroy_node()
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
