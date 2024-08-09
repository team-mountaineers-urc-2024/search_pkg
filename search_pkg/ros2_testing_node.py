import sys
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray

from robot_interfaces.msg import UrcCustomPath
from robot_interfaces.msg import UrcCustomPoint
from robot_interfaces.srv import GeneratePointsAround
from robot_interfaces.srv import GeneratePoints
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time

import numpy as np


class SearchTestingNode(Node):

    def __init__(self):
        super().__init__('search_testing_node')
        
        # self.pub_generate_point = self.create_publisher(UrcCustomPoint, 'generate_point', 10)
        # self.pub_generate_flag = self.create_publisher(Int8, 'generate_flag', 10)
        self.generate_around_cli = self.create_client(GeneratePointsAround, 'GeneratePointsAround')
        while not self.generate_around_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqGenAround = GeneratePointsAround.Request()

        self.generate_cli = self.create_client(GeneratePoints, 'GeneratePoints')
        while not self.generate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqGen = GeneratePoints.Request()

        #Resets the algorithm    
        # self.pub_reset_flag = self.create_publisher(Int8, 'reset_flag', 10)
        
        #Displays the algorithm    
        self.pub_display_flag = self.create_publisher(Int8, 'display_flag', 10)

        # #Tests the algorithm    
        # self.pub_test_self_update = self.create_publisher(Int8, 'test_self_update', 10)

        #Updates the PDF   
        self.pub_search_point = self.create_publisher(UrcCustomPoint, 'search_point', 10)


        self.sub_point_array = self.create_subscription(
            UrcCustomPath,
            'point_array',
            self.sub_point_array_callback,
            10)
        self.sub_point_array  # prevent unused variable warning

        self.gen_points = []   
        self.traversed = 0


        time.sleep(1) #Stall 1 second to allow other node to setup
        #Initiate test with first topic
        search_position = UrcCustomPoint()
        search_position.location_label = "Rover_test_position"
        search_position.point = PointStamped() #Im not giving it a header, thats just too much
        search_position.point.header.stamp = self.get_clock().now().to_msg() #Get current time
        search_position.point.point.x = 25.74
        search_position.point.point.y = 69.0
        search_position.point.point.z = 0.0 #Not used in waypoints



        #self.test_pos = search_position
        #self.pub_generate_point.publish(search_position)
        path = self.send_Generate_Around_request(search_position, 15.0)
        self.get_logger().info("SEARCH_TEST: " + str(len(path.path.points)))

        self.iterate_through_points(path.path.points)


    #Testing ability to recieve points from search_node
    def sub_point_array_callback(self, msg):
        self.get_logger().info("SEARCH_TEST: " + str(len(msg.path.points)))

        #Go through points
        self.iterate_through_points(msg.points)

    def send_Generate_Around_request(self, center, radius):
        self.reqGenAround.center = center
        self.reqGenAround.radius = radius
        self.future = self.generate_around_cli.call_async(self.reqGenAround)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def send_Generate_request(self):
        self.future = self.generate_cli.call_async(self.reqGen)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    #Testing Interface by updating PDF - Assuming Rover Teleports to each point   
    def iterate_through_points(self, points):
        
        if(self.traversed == 0 ):
            for i in range(len(points)):
                self.pub_search_point.publish(points[i])
                self.get_logger().info("Published Point: ["+ str(points[i].point.point.x) + " , " + str(points[i].point.point.y) + "]")
                time.sleep(0.1)
            self.get_logger().info("Finished first iteration")

            self.traversed = 1
            #gen_more_flag = Int8()
            #gen_more_flag.data = 0
            #self.pub_generate_flag.publish(gen_more_flag)
            response = self.send_Generate_request()
            self.iterate_through_points(response.path.points)
            self.get_logger().info("Requested more points...")

        else:
            for i in range(len(points)):
                self.pub_search_point.publish(points[i])
                self.get_logger().info("Published Point: ["+ str(points[i].point.point.x) + " , " + str(points[i].point.point.y) + "]")
                time.sleep(0.1)

            self.get_logger().info("Finished second iteration")  

            
            time.sleep(15)
            display_flag = Int8()
            display_flag.data = 1
            self.pub_display_flag.publish(display_flag)
            


#Entry Point
def main(args=None):
    rclpy.init(args=args)

    test_node = SearchTestingNode()

    rclpy.spin(test_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()