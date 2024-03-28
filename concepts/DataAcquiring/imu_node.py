import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import time
import numpy as np
import sys
import serial

#from std_msgs.msg import Double
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Empty


class IMUNode(Node):

    imu_number = [3]
    streaming = False
    

    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Vector3, 'imu/angles', 1)
        timer_period = 0.005 # seconds 1/fs
        cb_group =  ReentrantCallbackGroup()
        
        self.serial_port = serial.Serial(port="/dev/ttyACM0", baudrate=115200, timeout=0.01)
        
        self.service_start = self.create_service(Empty, 'imu/start', self.start_streaming,callback_group=cb_group)
        self.service_stop = self.create_service(Empty, 'imu/stop', self.stop_streaming)
        self.service_tare = self.create_service(Empty, 'imu/tare', self.tare_sensor)
        self.service_offset = self.create_service(Empty, 'imu/offset', self.offset_sensor)
        self.service_base_offset = self.create_service(Empty, 'imu/base_offset', self.base_offset_sensor)

        

        # Configure streaming slots, just the first with euler angles(1), the others with nothing (255)
        self.set_streaming_slots([1, 255, 255, 255, 255, 255, 255, 255]) 
        self.set_streaming_timing(100)

        # Set euler decomposition
        for id in self.imu_number:
                command = self.create_imu_command(id, 16,[5])
                self.apply_command(command)

        # Disable compass (109)
        for id in self.imu_number:
                command = self.create_imu_command(id, 109,[0])
                self.apply_command(command)

        self.manual_flush()

        self.timer = self.create_timer(timer_period, self.loop_callback,callback_group=cb_group)


    def loop_callback(self):
        #self.get_logger().info("Timer callback")
        if self.streaming:
            bytes_to_read = self.serial_port.inWaiting()
            if bytes_to_read > 0:
                data = self.serial_port.read(bytes_to_read)
                if data[0] == 0 and len(data) > 3:
                    decoded_data = data.decode()
                    #print("Valores: ")
                    
                    list_data = decoded_data.replace('\r\n',' ').split(' ')
                    
                    cleaned_list_data = list(filter(None, list_data))
                    #print(cleaned_list_data)
                    index = len(cleaned_list_data) -1
                    euler_vector = cleaned_list_data[index][3:].split(',')          
                    euler_vector = np.array(euler_vector, dtype=np.float64)
                    
                    # pitch, yaw, roll
                    msg = Vector3()
                    msg.x = euler_vector[0]
                    msg.y = euler_vector[1]
                    msg.z = euler_vector[2]
                    # msg.x = 0.0#rad2deg(euler_vector[0])
                    # msg.y = self.rad2deg(euler_vector[1])
                    # msg.y = self.fix_angle_quadrant(msg.y)
                    # msg.z = 0.0#rad2deg(euler_vector[2])
                    # msg = Float64()
                    # msg.data = self.rad2deg(euler_vector[1])
                    # msg.data = self.fix_angle_quadrant(msg.data)

                    self.publisher_.publish(msg)

    def manual_flush(self):
        """ Clean serial port buffer
        """
        while not self.serial_port.inWaiting() == 0:
            self.serial_port.read(self.serial_port.inWaiting())

    def create_imu_command(self,logical_id, command_number, arguments = []):
        """ Create imu command string

        Args: 
            logical_id: integer represents imu sensor configure to dongle 
            (configure in sensor suit)
            command_number: integer represents a command (see all commands in
            user manual)
            arguments: list with arguments, if necessary
        
        Return:
            encoded string with the command in the correct format
        """
        # Create command
        command = ">"+str(logical_id)+","+str(command_number)
        if(len(arguments) != 0):
            arguments_string = ","
            for  argument in arguments:
                arguments_string += str(argument)
                arguments_string += ","
            arguments_string = arguments_string[:-1]
            command += arguments_string
        command += '\n'
        return command.encode()
    
    def apply_command(self, command, showResponse=False):
        """ Apply command in sensor

        Args:
            serial_port: PySerial Object
            command: encoded string with the command
            showResponse: boolean that decides if output will be displayed
        """
        self.serial_port.write(command)
        time.sleep(0.1)
        if(showResponse):
            while self.serial_port.inWaiting():
                out = '>> ' + self.serial_port.read(self.serial_port.inWaiting()).decode()
            print(out)
        time.sleep(0.1)

    def stop_streaming(self,request,response):
        """ Apply stop streaming operation

        """

        for id in self.imu_number:
            command = self.create_imu_command(id, 86)
            self.apply_command(command)
        self.streaming = False
        return response

    def start_streaming(self,request,response):
        """ Apply start streaming operation

        """
        print("Start Streaming")
        for id in self.imu_number:
            command = self.create_imu_command(id, 85)
            self.apply_command(command)
        self.streaming = True
        return response
    
    def shutdown_sequence(self):
        self.stop_streaming({},{})
        self.serial_port.close()

    def set_streaming_slots(self, commands):
        """ Set streaming slots

        Args:
            serial_port: PySerial Object
            logical_ids: list of sensors that the command should be applied
            commands: list of integer that commands slots should be filled
        """
        for id in self.imu_number:
            command = self.create_imu_command(id, 80, commands)
            print(command)
            self.apply_command(command, True)

    def set_streaming_timing(self, freq):
        """ Set streaming slots

        Args:
            serial_port: PySerial Object
            logical_ids: list of sensors that the command should be applied
            commands: list of integer that commands slots should be filled
        """
        for id in self.imu_number:
            # comando argumentos: tempo entre amostras us (freq), tempo para amostrar 255 continuo, delay para amostragem 0
            if freq > 0:
                command = self.create_imu_command(id, 82, [int(1000000/freq),-1,0])
            else:
                command = self.create_imu_command(id, 82, [0,-1,0])
            print(command)
            self.apply_command(command, True)
        
    
    def tare_sensor(self,request,response):
        """ Apply tare sensor operation and add base Offset

        """
        # Tare with current orientation (96)
        for id in self.imu_number:
            command = self.create_imu_command(id, 96)
            self.apply_command(command)
        # Set base offset (22)
        #for id in imu_number:
        #    command = create_imu_command(id, 22)
        #    apply_command(serial_port, command)
        return response
    
    def rad2deg(self,rad):
        deg = (180.0*(rad)/np.pi)
        return deg

    def fix_angle_quadrant(self,deg):
        if deg < 0:
            deg+=360.0
        return deg

    def tare_sensor_quaternion(self):
        """ Apply tare sensor operation and add base Offset

        """
        # Tare with current orientation (96)
        for id in self.imu_number:
            command = self.create_imu_command(id, 97,[])
            self.apply_command(command)
        # Set base offset (22)
        #for id in imu_number:
        #    command = create_imu_command(id, 22)
        #    apply_command(serial_port, command)
        return {}

    def base_offset_sensor(self,request,response):
        """ Apply tare sensor operation and add base Offset

        """

        # Tare with current orientation (96)

        # Set base offset (22)
        for id in self.imu_number:
            command = self.create_imu_command(id, 22)
            self.apply_command(command)
        return response

    def offset_sensor(self,request,response):
        """ Apply offset sensor operation

        """
        # Offset with current orientation (19)
        for id in self.imu_number:
            command = self.create_imu_command(id, 19)
            self.apply_command(command)
        return response    




def main(args=None):
    rclpy.init(args=args)

    imu_node = IMUNode()
    #imu_node.create_timer(0.005,imu_node.timer_callback)

    try:
        imu_node.get_logger().info('Starting spin...\n')
        executor = MultiThreadedExecutor()
        executor.add_node(imu_node)
        executor.spin()
    #rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Keyboard interrupt, shutting down.\n')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_node.stop_streaming({},{})
    #imu_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
