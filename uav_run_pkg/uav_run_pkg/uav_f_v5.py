#!/usr/bin/env python3

import json
import rclpy
import time  #this was for sleep() but the other process are stoped apparently. I created own self.sleep 

from rclpy.node import Node
#from example_interfaces.msg import Int64
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
from mavros_msgs.srv import CommandInt # for any mavlink command, like ROI
from sensor_msgs.msg import NavSatFix # for global_postition/global
from sensor_msgs.msg import TimeReference #for gps time
from pygeodesy.geoids import GeoidPGM # to measure correct altitude
from std_msgs.msg import String # used to send messages to mqtt

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

class MiNodo(Node):
    def __init__(self):
        super().__init__("uav") # type: ignore # node name
        #  define here the namespace instead of /mavros/, example for the following launch:
        #  ros2 launch mavros apm.launch dev_url:=ip_address:15200 namespace:=hola 
        #  self.namespace = "/hola"
        self.namespace = "/uavLeader"  
        self.get_logger().info("Nodo uav creado")
        self.time_0         = 0
        self.time_flight_ini= 0
        self.current_state  = State()
        self.c_pose         = Odometry()
        self.global_position = NavSatFix()
        self.waypoint_global = GeoPoseStamped()
        self.MAS_position   = GeoPoseStamped() # my position shared to others in the MultiAgentSystem
        self.wp_from_mqtt   = GeoPoint()
        self.mqtt_msg       = String()
        self.alt0           = 0.0 # initial altitude for taking off
        self.alt_after_T_Off_geoid=0 # initial altitude in geoidesic after takeoff
        self.floor_geoid    = 0.0
        self.alt_AMSL       = 0.0
        self.frec           = 100.0 # (Hz), con esto, spin_once hace un delay de 10ms
        self.looprate       = self.create_rate(self.frec, self.get_clock())
        self.time_gps       = TimeReference()
        self.zkms_flag       = False
        #self.publisher2_ = self.create_publisher(Int64,"number_count",2)
        #self.create_timer( 1 , self.publicar )
        self.cont = 0
        
        #publisher setpoint_position/global topic
        self.gl_pos_pub = self.create_publisher(
            GeoPoseStamped,
            self.namespace+"/setpoint_position/global",
            qos_profile=SENSOR_QOS
        )

        #publisher uav1/GPSposition topic (este MAS era para compartir con otros UAVs pero no es necesario)  
        self.MAS_pub = self.create_publisher(
            GeoPoseStamped,
            "/uav1/GPSposition",
            qos_profile=SENSOR_QOS
        )

       #publisher position sent to mqtt
        self.ros2mqtt_pos_pub = self.create_publisher(
            String,
            "ros2mqtt/btu/drone/position",
            1 
        )

       #publisher position sent to mqtt
        self.ros2mqtt_status_pub = self.create_publisher(
            String,
            "ros2mqtt/btu/drone/status",
            1 
        )

        #self.subscriber_=self.create_subscription(
        #    Int64, 
        #    "number",
        #    self.callback_subscribir, 
        #    2
        #)
        ###################################
        # SUBSCRIBERS
        ###################################
                #subscriber to the local position
        self.pos_sub=self.create_subscription(
            Odometry, 
            self.namespace+"/global_position/local", 
            self.pos_cb, 
            qos_profile=SENSOR_QOS
        )

        #subscriber to the globl position
        self.gl_pos_sub=self.create_subscription(
            NavSatFix, 
            self.namespace+"/global_position/global", 
            self.global_position_cb, 
            qos_profile=SENSOR_QOS
        )
        # subscriber to the state of the uav
        self.state_sub=self.create_subscription(
            State,
            self.namespace+"/state", #ojo
            self.state_cb,
            1
        ) 
        #subscriber to the time of the gps
        self.time_ref_sub=self.create_subscription(
            TimeReference,
            self.namespace+"/time_referece", 
            self.time_ref_cb,
            1
        ) 
        #subscriber to the zkms via mqtt
        self.zkms_comands_sub=self.create_subscription(
            String,
            "/mqtt2ros/btu/drone/commands", 
            self.zkms_commands_cb,
            1
        ) 
                 
        #self.create_timer( 10.0 , self.timer_callback)
        self.counter_=0

    def new_method(self):
        return GeoPoseStamped
      
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
        time_stamp = self.global_position.header.stamp
        
        #self.get_logger().info("lat lon alt="+str(lat)+", "+str(lon)+", "+str(alt))
        timeROS = self.get_clock().now().to_msg()
        #self.get_logger().info("Elapsed time ="+str(time_1) )
        
        #self.mqtt_msg.data="data: lat lon alt="+str(lat)+","+str(lon)+","+str(alt)
        self.mqtt_msg.data='{"name":"drone1","lat":'+str(lat)+',"lon":'+str(lon)+',"alt":'+str(alt)+',"timeROS":'+str(time_stamp.sec)+"."+str(time_stamp.nanosec)+',"timeGPS":'+str(self.time_gps.time_ref.sec)+"."+str(self.time_gps.time_ref.nanosec)+"}"# timeROS gives the same time as in the global_position, variations of 1ms approx, is the same?
        
        milisec=str(int(time_stamp.nanosec/1e6)).zfill(3)
        alt = round(alt,1)
        if (lat==0.0) & (lon==0.0):
            #self.mqtt_msg.data='{"name":"drone1","lat":---,"lon":---,"alt":---,"timeStmp":'+str(time_stamp.sec)+milisec+"}"# timeROS gives the same time as in the global_position, variations of 1ms approx, is the same?
            self.mqtt_msg.data='{"name":"drone1","lat":"---","lon":"---","alt":"---","timeStmp":'+str(time_stamp.sec)+milisec+"}"# timeROS gives the same time as in the global_position, variations of 1ms approx, is the same?
            #self.mqtt_msg.data='{"name":"drone1","lat":'+str(lat)+',"lon":'+str(lon)+',"alt":'+str(alt)+',"timeStmp":'+str(time_stamp.sec)+milisec+"}"# timeROS gives the same time as in the global_position, variations of 1ms approx, is the same?
        else:
            self.mqtt_msg.data='{"name":"drone1","lat":'+str(lat)+',"lon":'+str(lon)+',"alt":'+str(alt)+',"timeStmp":'+str(time_stamp.sec)+milisec+"}"# timeROS gives the same time as in the global_position, variations of 1ms approx, is the same?
        #self.mqtt_msg.data='{"name":"drone1","lat":'+str(lat)+',"lon":'+str(lon)+',"alt":'+str(alt)+',"timeGPS":'+str(time_stamp.sec)+"."+str(time_stamp.nanosec)+"}"
        self.ros2mqtt_pos_pub.publish(self.mqtt_msg)
           
        
    #def callback_subscribir(self,msg):
    #    #valor=str(msg.data)
    #    self.get_logger().info(str(msg.data))
    #    self.cont+=msg.data
    #    print("esto vale contador: {}" . format(self.cont))
    #    var64= Int64()
    #    var64.data = self.cont
    #    self.publisher2_.publish(var64)
    
    def state_cb(self,msg):
        self.current_state = msg     

    def time_ref_cb(self,msg):
        self.time_gps = msg

    def zkms_commands_cb(self,msg):
        self.get_logger().warn("zkms command receibed by Mqtt")
        mqtt_command = msg
        jsondata = mqtt_command.data
        fields = json.loads(jsondata)
        lat = fields["lat"]
        lon = fields["lon"]
        alt = fields["alt"]
        #self.wp_from_mqtt=[lat lon alt]
        self.wp_from_mqtt.latitude  =lat
        self.wp_from_mqtt.longitude =lon
        self.wp_from_mqtt.altitude  =alt
        self.get_logger().info("New position: [lat, lon, alt]=[ "+str(lat)+", "+str(lon)+", "+str(alt)+" ]")
        self.zkms_flag = True #this will be replaced by the real command, how to extract the fields ??
        status = String()
        status.data = '{"status":"Going to new position"}'
        self.ros2mqtt_status_pub.publish(status)

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
            
    # SERVICES:
    def arming(self):
        client_a = self.create_client(CommandBool, self.namespace+"/cmd/arming")
        #response = 1
        #while response != 0 and self.current_state.mode == "GUIDED":
        while not client_a.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service *arming* not available, waiting again...')
        req = CommandBool.Request()
        req.value = True
        future = client_a.call_async(req)
        future.add_done_callback(partial(self.call_arming_cb))
        self.get_logger().info("arming...")
        self.time_flight_ini = self.get_clock().now().to_msg().sec #the efective one is in take off, here is in case there is no takein off
        status = String()
        status.data = '{"status":"Arming drone"}'
        self.ros2mqtt_status_pub.publish(status)
              #response = future.result
            #if response !=0:
            #    self.get_logger().info("response to arm(): "+str(response))
            #    self.sleep(5)
        
        

    def call_arming_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service 'arming' call failed %r" % (e))
        self.get_logger().info("response to arm(): "+str(response))
        
    
    def takingoff(self, altitude):
        client_t = self.create_client(CommandTOL, self.namespace+"/cmd/takeoff")
        if self.current_state.mode == "GUIDED":
            while not client_t.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('service *takeoff* not available, waiting again...')
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.altitude = altitude # based on the example mavros/issues/1718
            req.yaw = 0.0
            future = client_t.call_async(req)
            future.add_done_callback(partial(self.call_takingoff_cb))
            self.get_logger().info("takingoff...")
            self.time_flight_ini = self.get_clock().now().to_msg().sec
            print("Init flight time: {}".format(self.time_flight_ini))
            status = String()
            status.data = '{"status":"Take off iniciated"}'
            self.ros2mqtt_status_pub.publish(status)    

    def call_takingoff_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service 'takingoff' call failed %r" % (e,))
        self.get_logger().info("response to take off: "+str(response))

    def landing(self):
        client_l = self.create_client(CommandTOL, self.namespace+"/cmd/land")
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
        self.get_logger().info("response to landing: "+str(response))

    def start_stream(self):
        """function will start the stream of global_position/local and others. This is necessary for Real Time flight instead of Gazebo
        Returns:
                0 (int):  successful.
                -1 (int):  unsuccessful.        """
        client = self.create_client(StreamRate, self.namespace +"/set_stream_rate")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service set_stream_rate not available, trying again...')
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 1 # Hz, the maximum I saw was like 7Hz
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
        self.get_logger().info("response from stream service: "+str(response))

    def ROI(self, lat, lon, alt):
        """ ROI-region of interest (mavlink command 195) 
         """
        client = self.create_client(CommandInt, self.namespace +"/cmd/command_int")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('mavlink service comand_int not available, trying again...')
        req = CommandInt.Request()
        req.command = 195 # 195=MAV_CMD_DO_SET_ROI_LOCATION (mavlink documentation)
        req.frame = 6 # reference frame xyz = lat, lon, altitude relative to home
        req.param2 =0.0 # gimbal device ID
        req.param2 =0.0
        req.param3 =0.0
        req.param4 =0.0 
        req.x = (int)(lat*1e7) #517691000  #latitude
        req.y = (int)(lon*1e7) #143235800  #longitud
        req.z = alt #5.0        #altitude relative to home (float)
        #self.get_logger().info("xyz_ROI = "+str(req.x)+", "+str(req.y)+", "+str(req.z))
        future = client.call_async(req)
        future.add_done_callback(partial(self.ROI_cb))

    def ROI_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        self.get_logger().info("response to ROI: "+str(response))

    def ROI_off(self):
        """ cancel ROI-region of interest (mavlink command 197) 
         """
        client = self.create_client(CommandInt, self.namespace +"/cmd/command_int")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('mavlink service comand_int not available, trying again...')
        req = CommandInt.Request()
        req.command = 197 # 197=MAV_CMD_DO_SET_ROI_NONE (mavlink documentation)
        req.frame = 6 # reference frame xyz = lat, lon, altitude relative to home
        req.param1 =0.0 # gimbal device ID
        req.param2 =0.0
        req.param3 =0.0
        req.param4 =0.0 
        future = client.call_async(req)
        future.add_done_callback(partial(self.ROI_off_cb))

    def ROI_off_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        self.get_logger().info("response to ROI_off: "+str(response))

    def Loiter_turns(self,lat, lon, alt, num_turns, radius):
        """ do circles around a position (mavlink command 18)
        args: lat, lon, alt, num_turns, radius 
         """
        client = self.create_client(CommandInt, self.namespace +"/cmd/command_int")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('mavlink service comand_int not available, trying again...')
        req = CommandInt.Request()
        req.command = 18 # 197=MAV_CMD_DO_SET_ROI_NONE (mavlink documentation)
        req.frame = 6 # reference frame xyz = lat, lon, altitude relative to home
        req.param1 =(float)(num_turns) # number of turns
        req.param2 =0.0 #leave circle only once heading towards next wp
        req.param3 =(float)(radius) 
        req.param4 =0.0  #Xtrack location
        req.x = 517691000#(int)(lat*1e7) #517691000  #latitude
        req.y = 143235800#(int)(lon*1e7) #143235800  #longitud
        req.z = 20.0#(float)(alt) #5.0        #altitude relative to home (float)
        future = client.call_async(req)
        future.add_done_callback(partial(self.Loiter_turns_cb))

    def Loiter_turns_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        self.get_logger().info("response to Loiter_turns: "+str(response))

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
        self.alt_AMSL = alt +  self.floor_geoid - self.geoid_hight(lat,lon)
        #self.waypoint_global.pose.position = GeoPoint(lat,lon,alt_AMSL) this didnt work for me, I use the following 3
        self.waypoint_global.pose.position.altitude = self.alt_AMSL
        self.waypoint_global.pose.position.longitude= lon
        self.waypoint_global.pose.position.latitude = lat
        self.gl_pos_pub.publish(self.waypoint_global)

    def sleep(self,secs):
        #minimum delay is 1/frec = 10ms, this is, secs = 0.01
        ii = 0
        while ii<int(self.frec*secs): #delay s
            rclpy.spin_once(self)
            ii+=1

    def flight_time(self): #return the elapsed time measured since the take off (seconds)
        dif = self.get_clock().now().to_msg().sec - self.time_flight_ini
        self.get_logger().info("elapsed time since takeoff:"+str(dif)+" sec")
        return dif
    

def main(args=None):
    #initializing ROS node
    rclpy.init(args=args)

    # creation of node
    node=MiNodo()
    
    v=1
    print("main version ={}" . format(v))
    node.time_0 = node.get_clock().now().to_msg().sec
    print("Init time: {}".format(node.time_0))

    
    #request service stream rate start
    node.start_stream()
    
    # Wait for FCU connection.
    node.wait4connect()

    # Wait for the mode to be switched to GUIDED.
    #node.wait4start()
    
    #request service stream rate start
    node.start_stream()
    
    #request service arming
    node.arming()
    node.sleep(1.0) 

    #request takeoff
    node.alt0 = 3.0  # floating point necessary
    print("Geod altitude before takeoff: {}".format(node.global_position))
    node.takingoff(node.alt0) 
    node.sleep(8.0) 
    print("Geod altitude after  takeoff: {}".format(node.global_position))

    #calculate the floor compensation
    # node.alt_after_T_Off_geoid = node.global_position.altitude
    # node.floor_geoid = node.alt_after_T_Off_geoid-node.alt0
    node.floor_geoid = node.global_position.altitude
    print("Floor in geoid is: {}".format(node.floor_geoid))
    
   
    # rospy.Rate(1) # in ros2 this is done with the loop_rate but not sure how, I can use the sleep() that I made

    # Specify some global waypoints
    """goals =[[51.76910,14.32358,node.alt0],  # WP 1
            [51.76913,14.32358,node.alt0],  # WP 2
            [51.76913,14.32355,node.alt0],  # WP 3
            [51.76910,14.32355,node.alt0],  # WP 4
            [51.76910,14.32358,node.alt0]]  # Back to WP 1"""
    lat_ini = 51.76910
    lon_ini = 14.32358 
    square_long = 0.00003 # in geo coordinates (*e7 aprox metters)
    goals =[[lat_ini, lon_ini,node.alt0],  # WP 1
            [lat_ini+square_long, lon_ini,node.alt0],  # WP 2
            [lat_ini+square_long, lon_ini-square_long, node.alt0],  # WP 3
            [lat_ini, lon_ini-square_long, node.alt0]]  # WP 4

    ii = 1
    while (ii <= 10) & (node.zkms_flag==False): #repeat 10 times the square
        i = 0  
        contador = 0
        while (i < len(goals)) & (node.zkms_flag==False): 

            lat = goals[i][0]
            lon = goals[i][1]
            alt = goals[i][2]
            node.set_global_destination(lat,lon,alt) # alt wrt floor 
            
            # measured coords:
            c_lat = node.global_position.latitude
            c_lon = node.global_position.longitude
            c_alt = node.global_position.altitude
            print("Current  global LLA: {},  {} , {}".format(c_lat,c_lon,c_alt))
            print("Setpoint global LLA: {},  {} , {}".format(lat,lon,node.alt_AMSL))
            
            node.MAS_position.pose.position.longitude= c_lon
            node.MAS_position.pose.position.latitude = c_lat        
            node.MAS_position.pose.position.altitude = c_alt-node.floor_geoid
            node.MAS_pub.publish(node.MAS_position)
            
            #time.sleep(1) # this is from library "time" but blocks the subscriptions, there is a better way to code using "Actions" in ros, (to be done)
            node.sleep(1.0)
            contador+= 1
            if node.flight_time() >= 61:
                node.zkms_flag = True
                print("received zkms position at  t= {}s". format(node.flight_time()))
            if contador ==7: #wait 7 seconds to update wp
                print("Doing square number: {}".format(ii))
                contador = 0
                #if i == 1:
                    #node.Loiter_turns(51.76918,14.32358,10.0,3,10)
                    #node.ROI(51.769115,14.323565,10.0) #request service ROI mavlink command
                    #node.sleep(1.0)
                i+=1
        ii+=1
    ii = 1
    while (ii <= 120):
        if node.zkms_flag==True: # go to zkms position
            print("Waypoints interrupted by zkms!")
            print("Going to new position")
            node.zkms_flag= False
            lat = lat_ini+0.00009
            lon = lon_ini+0.00009
            alt = node.alt0
            node.set_global_destination(lat,lon,alt) # first test without mqtt 
            node.set_global_destination(node.wp_from_mqtt.latitude,
                                        node.wp_from_mqtt.longitude,
                                        node.wp_from_mqtt.altitude) # test with mqtt 
            node.sleep(20)
            lat = lat_ini
            lon = lon_ini
            alt = node.alt0
            node.set_global_destination(lat,lon,alt) # alt wrt floor 
            node.sleep(10)
        else:
            print("Waypoints completed!")
        node.sleep(1)
        ii+=1

    #node.Loiter_turns(51.76918,14.32358,5.0,3,200) #alt lon alt num radious
    #node.ROI_off() # cancel any previous ROI
    #node.sleep(10)
    #node.ROI(51.76914,14.32360,5.0)
    #node.sleep(10)
    #node.ROI_off
    #node.sleep(10)
    #node.ROI(51.76920,14.32358,5.0)
    node.sleep(5)

    #time_1 = node.get_clock().now().to_msg().sec
    #time_2= 0
    #while time_2 < time_1+20: #secs
    #    time_2 = node.get_clock().now().to_msg().sec
    #    node.sleep(1)
       
    # request service landing
    node.landing()

    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__=="__main__":
    main()