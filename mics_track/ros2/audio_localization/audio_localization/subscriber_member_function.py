import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray
import numpy as np
from std_msgs.msg import String

#Spectrum is saved here
buff = []
#List of yaw commands to execute to find heading
yaw_commands = []
#RMS value for previous variable. Size is equal to yaw_commands
overall_rms_values = []
#Used for publishing to dynamixel 
yaw_angle = Float32MultiArray()
#Range of motion of Dynamixel for detecting the sound heading
current_range = [0,360.0]
#Reduction rate of above variable at every heading discovery loop
RANGE_REDUCE_FACTOR = 0.25
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
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/audio/spectrum', 10)
        timer_period = 0.033  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self._loop_rate = self.create_rate(0.33, self.get_clock())
        self.i = 0

    def timer_callback(self):
         if(len(yaw_angle.data)>0):
            self.publisher_.publish(yaw_angle)
            yaw_angle.data.clear()
            self.get_logger().info('Publish and sleep') #Dynamixel makes noise
            self._loop_rate.sleep()
            self.i += 1

            buff.clear()

class AudioSubscriber(Node):

    def __init__(self):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/audio/spectrum',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('[Spectrum] I heard: "%s"' % msg.data)
        #buff.append(list(msg.in_data))

class ClassificationSubscriber(Node):

    def __init__(self):
        super().__init__('classification_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/audio/classification',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('[Classification] I heard: "%s"' % msg.data)
        buff.append(list(msg.data)[1:]) #Firt element is the enumeration of the class the sound belongs to
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
            yaw_angle.data.append(yaw_commands[len(overall_rms_values)])
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
        #Dynamixel makes sound
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
        print(yaw_commands)
        print("RESTART!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    audio_subscriber = AudioSubscriber()
    classification_subscriber= ClassificationSubscriber()

    try:
        rclpy.spin(audio_subscriber)
        rclpy.spin(classification_subscriber)
    except KeyboardInterrupt:
        print("Shutting down")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
