#!/usr/bin/env python3
import rclpy
import time  #this is for sleep

from rclpy.node import Node
from example_interfaces.msg import Int64
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from functools import partial # this is to the SstreamRate service call

# list of topics and messages wiki.ros.org/mavors
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from nav_msgs.msg import Odometry  
from mavros_msgs.srv import StreamRate  # to be able to receive something in global_position/local 
from mavros_msgs.srv import CommandBool # this works for cmd/arming 
from mavros_msgs.srv import CommandTOL # this works for cmd/land and cmd/takeoff

from mavros.base import SENSOR_QOS  # this is for the quality of singal mismatch in the local pos
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import SetMode
from sensor_msgs.msg import NavSatFix # for global_postition/global
from pygeodesy.geoids import GeoidPGM # to measure correct altitude



_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

class MiNodo(Node):
    def __init__(self):
        super().__init__("uav") # type: ignore # node name
        self.get_logger().info("Nodo uav creado")
        self.current_state = State()
        self.c_pose = Odometry()
        self.global_position = NavSatFix()
        self.waypoint_global = GeoPoseStamped()
        self.MAS_position = GeoPoseStamped() # my position shared to others in the MultiAgentSystem
        self.alt0 = 0 # initial altitude for taking off
        self.alt_after_T_Off_geoid=0 # initial altitude in geoidesic after takeoff
        self.floor_geoid =0
        frec = 100.0 # (Hz), con esto, spin_once hace un delay de 10ms
        self.looprate = self.create_rate(frec, self.get_clock())

        self.publisher2_ = self.create_publisher(Int64,"number_count",2)
        #self.create_timer( 1 , self.publicar )
        self.cont = 0

        self.gl_pos_pub = self.create_publisher(
            GeoPoseStamped,
            "/mavros/setpoint_position/global",
            qos_profile=SENSOR_QOS
        )

        self.MAS_pub = self.create_publisher(
            GeoPoseStamped,
            "/uav1/GPSposition",
            qos_profile=SENSOR_QOS
        )        

        self.subscriber_=self.create_subscription(
            Int64, 
            "number",
            self.callback_subscribir, 
            2
        )

        #subscriber to the local position
        self.pos_sub=self.create_subscription(
            Odometry, 
            "/mavros/global_position/local", 
            self.pos_cb, 
            qos_profile=SENSOR_QOS
        )

        #subscriber to the globl position
        self.gl_pos_sub=self.create_subscription(
            NavSatFix, 
            "/mavros/global_position/global", 
            self.global_position_cb, 
            qos_profile=SENSOR_QOS
        )

        self.state_sub=self.create_subscription(
            State,
            "/mavros/state", #ojo
            self.state_cb,
            1
        ) 
        #self.create_timer( 10.0 , self.timer_callback)
        self.counter_=0
      
    def pos_cb(self,msg):
        #self.get_logger().info(str(msg.pose.pose.position)) #print the pose       
        self.c_pose = msg
        x = self.c_pose.pose.pose.position.x
        y = self.c_pose.pose.pose.position.y
        z = self.c_pose.pose.pose.position.z
        #self.get_logger().info("x y z ="+str(x)+", "+str(y)+", "+str(z)) #print the pose       
    
    def global_position_cb(self,msg):
        self.global_position = msg
        lat = self.global_position.latitude
        lon = self.global_position.longitude
        alt =  self.global_position.altitude
        #self.get_logger().info("lat lon alt="+str(lat)+", "+str(lon)+", "+str(alt))      
    
    def callback_subscribir(self,msg):
        #valor=str(msg.data)
        self.get_logger().info(str(msg.data))
        self.cont+=msg.data
        print("esto vale contador: {}" . format(self.cont))
        var64= Int64()
        var64.data = self.cont
        self.publisher2_.publish(var64)
    
    def state_cb(self,msg):
        self.current_state = msg     

    def timer_callback(self):
        self.counter_-=1
        for i in range(10):# va de 1 a 10
             self.get_logger().info("cont " +str(self.counter_))

    def wait4connect(self):
        """ Wait until status of uav is connected
        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        self.get_logger().info("Waiting for 'connected' status")
        while rclpy.ok() and not self.current_state.connected:
            rclpy.spin_once(self)
            #self.get_logger().info("Despues del loop sleep")        
        else:
            if self.current_state.connected:
                self.get_logger().info("Pixhawk connected")
                return 0
            else:
                self.get_logger().error("Error connecting Pixhawk")
                return -1


    def wait4start(self):
        """ Wait until pilot put uav in Guided (via RC or GCS)
        Returns:
                0 (int): guided.
                -1 (int): fail guided.
        """
        self.get_logger().info("Waiting for user to set mode to GUIDED")
        #while rclpy.ok() and not self.current_state.guided: #este guided es diferente al mode.GUIDED
        while rclpy.ok() and not self.current_state.mode == "GUIDED": #este guided es diferente al mode.GUIDED
            rclpy.spin_once(self)
        else:
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("mode =" +str(self.current_state.mode))
                self.get_logger().info("Mode set to GUIDED. Starting Mission...")
                return 0
            else:
                self.get_logger().error("mode =" +str(self.current_state.mode))
                self.get_logger().error("Error starting Mission...")
                return -1

    def arming(self):
        client_a = self.create_client(CommandBool, "/mavros/cmd/arming")
        while not client_a.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service *arming* not available, waiting again...')
        req = CommandBool.Request()
        req.value = True
        future = client_a.call_async(req)
        future.add_done_callback(partial(self.call_arming_cb))
        self.get_logger().info("arming...")

    def call_arming_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service 'arming' call failed %r" % (e,))

    def takingoff(self, altitude):
        client_t = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        while not client_t.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service *takeoff* not available, waiting again...')
        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.altitude = altitude # based on the example mavros/issues/1718
        req.yaw = 0.0
        future = client_t.call_async(req)
        future.add_done_callback(partial(self.call_takingoff_cb))
        self.get_logger().info("takingoff...")

    def call_takingoff_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service 'takingoff' call failed %r" % (e,))

    def landing(self):
        client_l = self.create_client(CommandTOL, "/mavros/cmd/land")
        while not client_l.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service landing not available, waiting again...')
        req = CommandTOL.Request()
        req.yaw = 0.0 # there are others like lat, lon, alt, i dont know if they are needed
        future = client_l.call_async(req)
        future.add_done_callback(partial(self.call_landing_cb))
        self.get_logger().info("landing...")

    def call_landing_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service 'landing' call failed %r" % (e,))

    def start_stream(self):
        """function will start the stream of global_position/local and others. This is necessary for Real Time flight instead of Gazebo
        Returns:
                0 (int):  successful.
                -1 (int):  unsuccessful.        """
        client = self.create_client(StreamRate, "/mavros/set_stream_rate")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service set_stream_rate not available, waiting again...')
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 10 # Hz, the maximum I saw was like 7Hz
        req.on_off = True
        #future is a functionality of python , not from  ros
        future = client.call_async(req)
        #the following add_done_callback is a functionallity of future to handole more services without crashing
        future.add_done_callback(partial(self.callback_call_stream))

    def callback_call_stream(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def geoid_hight(self, lat,lon):
        """Calculates AMSL to ellipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help you convert from meters 
        above mean sea level (AMSL) to meters above
        the WGS84 ellipsoid.
    	If you want to go from AMSL to ellipsoid height, add the value.
        To go from ellipsoid height to AMSL, subtract this value.
        setpoint_position/global uses AMSL
        global_position/global uses ellipsoid
         """
        return _egm96.height(lat,lon)
    
    def set_global_destination(self, lat, lon, alt):
        # alt is wrt floor, we convert to avobe mean sea level to publish it
        # alt_AMSL = alt + self.alt_after_T_Off_geoid - self.geoid_hight(lat,lon)
        alt_AMSL = alt +  self.floor_geoid - self.geoid_hight(lat,lon)
        #self.waypoint_global.pose.position = GeoPoint(lat,lon,alt_AMSL) this didnt work for me, I use the following 3
        self.waypoint_global.pose.position.altitude = alt_AMSL
        self.waypoint_global.pose.position.longitude= lon
        self.waypoint_global.pose.position.latitude = lat
        self.gl_pos_pub.publish(self.waypoint_global)

def main(args=None):
    #initializing ROS node
    rclpy.init(args=args)

    # creation of node
    node=MiNodo()
    
    v=1
    print("main version ={}" . format(v))
    
    # Wait for FCU connection.
    node.wait4connect()
    
    # Wait for the mode to be switched.
    node.wait4start()

    #request service stream rate start
    node.start_stream()

    #request service arming
    node.arming()

    #request takeoff
    node.alt0 = 3.0  # floating point necessary
    node.takingoff(node.alt0) 
    time.sleep(3)
    node.alt_after_T_Off_geoid = node.global_position.altitude
    node.floor_geoid = node.alt_after_T_Off_geoid-node.alt0
    print("Floor in geoid is: {}".format(node.floor_geoid))
        

    # rospy.Rate(1) # I dont know how to do the equivalent in ros2

    # Specify some global waypoints
    goals = [[51.76910,14.32358,3],   # WP 1
            [51.76914,14.32360,3],  # WP 2
            [51.76916,14.32351,3],    # WP 3
            [51.76911,14.32350,3],   # WP 4
            [51.76910,14.32358,3]]  # Back to WP 1
    i = 0  
    contador = 0
    while i < len(goals):

        lat = goals[i][0]
        lon = goals[i][1]
        alt = goals[i][2]

        node.set_global_destination(lat,lon,alt) # alt wrt floor
        
        # measured coords:
        c_lat = node.global_position.latitude
        c_lon = node.global_position.longitude
        c_alt = node.global_position.altitude
        alt_AMSL = alt +  (node.floor_geoid - node.geoid_hight(lat,lon))
        print("Current  global LLA: {},  {} , {}".format(c_lat,c_lon,c_alt))
        print("Setpoint global LLA: {},  {} , {}".format(lat,lon,alt_AMSL))
        
        node.MAS_position.pose.position.longitude= c_lon
        node.MAS_position.pose.position.latitude = c_lat        
        node.MAS_position.pose.position.altitude = c_alt-node.floor_geoid
        node.MAS_pub.publish(node.MAS_position)
        
        contador+= 1
        time.sleep(1) # this is from library time, there is a better way to code using Actions in ros, to be done
        if contador ==10:
            contador = 0
            i+=1
    print("Waypoints completed!")

    # request service landing
    node.landing()

    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()
