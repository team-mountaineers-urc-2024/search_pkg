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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

#from array import array

import search_pkg.mod_search_object as Search
import numpy as np

#To manage multiple search nodes, make a new search object for each area?

class SearchNode(Node):

    def __init__(self):
        super().__init__('search_node')

        #PARAMETERS
        # Declare and read parameters
        self.declare_parameter(
            name="grid_count",
            value=100,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Number of Grid tiles in the search algorithm",
            ),
        )

        self.declare_parameter(
            name="observation_radius",
            value=3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="The radius around the rover in which we are confident in the object detection algorithm.",
            ),
        )

        self.declare_parameter(
            name="observation_certainty",
            value=1.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="How confident we are in the detection of an object within the observation radius. (Should come from Owen)",
            ),
        )

        self.declare_parameter(
            name="n_sample",
            value=7,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="The number of points to generate in each batch (request)",
            ),
        )

        self.grid_count = (
            self.get_parameter("grid_count").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Grid Count: {self.grid_count}")

        self.observation_radius = (
            self.get_parameter("observation_radius").get_parameter_value().double_value
        )
        self.get_logger().info(f"Observation Radius: {self.observation_radius}")

        self.observation_certainty = (
            self.get_parameter("observation_certainty").get_parameter_value().double_value
        )
        self.get_logger().info(f"Observation Certainty: {self.observation_certainty}")

        self.n_sample = (
            self.get_parameter("n_sample").get_parameter_value().integer_value
        )
        self.get_logger().info(f"N Samples: {self.n_sample}")


        self.point_count = 0

        #Setup search object
        #Robot position, Radius of searching circle, samples per iteration
        self.search_manager = Search.Search_Object([0,0], 15.0)

        #Initialize display
        self.search_manager.display_init()

        #SERVICES
        self.generate_around_srv = self.create_service(GeneratePointsAround, 'GeneratePointsAround', self.sub_generate_around_callback)

        self.generate_srv = self.create_service(GeneratePoints, 'GeneratePoints', self.sub_generate_callback)


        #Searches a point provided
        self.sub_search_point = self.create_subscription(
            UrcCustomPoint,
            'search_point',
            self.search_point_callback,
            10)
        self.sub_search_point  # prevent unused variable warning


        #Flag used to display algorithm in matplotlab
        self.sub_display_flag = self.create_subscription(
            Int8,
            'display_flag',
            self.sub_display_flag_callback,
            10)
        self.sub_display_flag  # prevent unused variable warning


    def create_Urc_Point(self, name, x, y):
        newPoint = UrcCustomPoint()
        newPoint.location_label = name
        newPoint.point = PointStamped()
        newPoint.point.point.x = x
        newPoint.point.point.y = y
        newPoint.point.point.z = 0.0 #Not used in waypoints
        newPoint.point.header.stamp = self.get_clock().now().to_msg() #Get current time
        newPoint.point.header.frame_id = self.frame_id
        #Dont know seq or frame ID

        return newPoint


    def create_Urc_Point_simplified(self, x, y):
        name = "Generated_Point_" + str(self.point_count)
        self.point_count = self.point_count + 1
        return self.create_Urc_Point(name, x, y)


    def search_point_callback(self, msg):
        point = [msg.point.point.x, msg.point.point.y]
        self.search_manager.redist_point(point)

        self.search_manager.update_display()



    def sub_display_flag_callback(self, msg):
        self.search_manager.update_display()
        self.search_manager.display_alg()
        

    def sub_generate_around_callback(self, request, response):
        point = [request.center.point.point.x, request.center.point.point.y]
        self.frame_id = request.center.point.header.frame_id 
        #Applies parameters to search algorithm
        self.search_manager.reset_algrothim(center_pos=point, rad=request.radius, sample_size=self.n_sample, observation_certainy=self.observation_certainty, observation_radius=self.observation_radius, grid_count=self.grid_count) #CHANGE THE PARAMETERS HERE AS WELL
        
        # temp = Int8()
        # temp.data = 0
        response = self.sub_generate_callback(request, response) #param doesnt matter
        return response


    #Publish points to point_array for rover to receive
    def sub_generate_callback(self, request, response):

        self.get_logger().info('SEARCH_NODE: Generating points...')

        ordered_samples = self.search_manager.generate_points()

        #Ordered_samples is a np.array
        self.get_logger().info('SEARCH_NODE: Points from search manager:' + str(ordered_samples))

        path = UrcCustomPath()
        path.time_recieved = self.get_clock().now().to_msg() 
        points_list = []

        #self.get_logger().info('SEARCH_NODE: test:' + str(ordered_samples[1][3]))

        for i in range(ordered_samples.shape[1]):
            nextpoint = self.create_Urc_Point_simplified((ordered_samples[0])[i], (ordered_samples[1])[i])
            points_list.append(nextpoint)

        path.points = points_list

        #self.get_logger().info('SEARCH_NODE: Publishing path')
        response.path = path #self.pub_point_array.publish(path)
        return response
        #self.get_logger().info('SEARCH_NODE: Done publishing...')
        #self.search_manager.display_points()



#Entry point
def main(args=None):
    rclpy.init(args=args)

    search_node = SearchNode()

    rclpy.spin(search_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    search_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()