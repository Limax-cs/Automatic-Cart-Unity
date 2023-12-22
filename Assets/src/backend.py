#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from automatic_cart.msg import App, Backend
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
#from tf import tfMessage
import tf
import tf2_ros
import tf2_geometry_msgs

class Coordinates:

    def __init__(self, px: float, py: float, pz: float, rx: float, ry: float, rz: float):
        self.px = px
        self.py = py
        self.pz = pz
        self.rx = rx
        self.ry = ry
        self.rz = rz


class CartBackend:

    def __init__(self):

        rospy.init_node('cart_backend', anonymous=True)

        # Calibration params
        #self.origin = Coordinates(-73.5, 9.5, 1.0, 0.0, 0.0, 0.0) # Initial robot location wrt image center (oriented to +x)
        #self.factor = Coordinates(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
        #self.robot = Coordinates(-73.5, 9.5, 1.0, 0.0, 0.0, 0.0)
        self.origin = Coordinates(-41.5, -11.5, 1.0, 0.0, 0.0, 0.0) # Initial robot location wrt image center (oriented to +x)
        self.factor = Coordinates(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
        self.robot = Coordinates(-41.5, -11.5, 1.0, 0.0, 0.0, 0.0)
        self.image_size = (300.0, 220.0) # Image size
        self.map_origin = (10.0, 10.0) # Upper-left vertex of the map
        self.image_scale = 1.0
        #self.tflistener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Path
        self.pathx_app = []
        self.pathy_app = []
        self.pathx_map = []
        self.pathy_map = []
    
        # Robot
        #self.odom_reader()
        #self.imu_reader()
        #self.tf_reader()
        self.scan_reader()
        self.drive_controller()
        
        # App
        self.app_reader()
        self.app_reporter()

        # Robot Status
        self.status = "Stop"

        # Robot Params
        self.dist_threshold = 1.0
        self.P = 1.0 # Proportional
        self.I = 0.0 # Integrative
        self.D = 0.0 # Derivative
        self.linearvel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.angularvel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
    
    # Backend to Robot
        
    def odom_reader(self):
    
        #rospy.init_node('cart_odom_subscriber', anonymous=True)
        rospy.Subscriber("cart_diff_drive_controller/odom", Odometry, self.odom_callback)
        
        
    def odom_callback(self, data):
    
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity X: %f", data.twist.twist.linear.x)
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Y: %f", data.twist.twist.linear.y)
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Z: %f", data.twist.twist.linear.z)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity X: %f", data.twist.twist.angular.x)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Y: %f", data.twist.twist.angular.y)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Z: %f", data.twist.twist.angular.z)
        #rospy.loginfo(rospy.get_caller_id() + "Position X: %f", data.pose.pose.position.x)
        #rospy.loginfo(rospy.get_caller_id() + "Position Y: %f", data.pose.pose.position.y)
        #rospy.loginfo(rospy.get_caller_id() + "Position Z: %f", data.pose.pose.position.z)
        #self.odom_data = data

        # Update robot coordinates
        orientation_list = [data.pose.pose.orientation.x,
                            data.pose.pose.orientation.y,
                            data.pose.pose.orientation.z,
                            data.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.robot.px = data.pose.pose.position.x*self.factor.px + self.origin.px
        self.robot.py = data.pose.pose.position.y*self.factor.py + self.origin.py
        self.robot.pz = data.pose.pose.position.z*self.factor.pz + self.origin.pz
        self.robot.rx = roll*self.factor.rx + self.origin.rx
        self.robot.ry = pitch*self.factor.ry + self.origin.ry
        self.robot.rz = yaw*self.factor.rz + self.origin.rz

    def tf_update(self):

        try:
            # Wait for the transform between "/map" and "/base_footprint" frames
            data = self.tf_buffer.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(3.0))

            # It works :,)
            print("Translation:", data.transform.translation)
            #print("Rotation:", transform.transform.rotation)

            # Update the status of the robot
            orientation_list = [data.transform.rotation.x,
                                data.transform.rotation.y,
                                data.transform.rotation.z,
                                data.transform.rotation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            self.robot.px = data.transform.translation.x*self.factor.px + self.origin.px
            self.robot.py = data.transform.translation.y*self.factor.py + self.origin.py
            self.robot.pz = data.transform.translation.z*self.factor.pz + self.origin.pz
            self.robot.rx = roll*self.factor.rx + self.origin.rx
            self.robot.ry = pitch*self.factor.ry + self.origin.ry
            self.robot.rz = yaw*self.factor.rz + self.origin.rz
            if self.robot.rz > math.pi: # Convert the range to -pi to pi
                self.robot.rz = self.robot.rz - 2*math.pi

            print(f"Translation - x: {self.robot.px}, y: {self.robot.py}, z: {self.robot.pz} | Rotation - roll: {self.robot.rx}, pitch: {self.robot.ry}, yaw: {self.robot.rz}")

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))

    def imu_reader(self):
    
        rospy.Subscriber("cart_imu", Imu, self.imu_callback)
        
        
    def imu_callback(self, data):

        # Update robot coordinates
        orientation_list = [data.orientation.x,
                            data.orientation.y,
                            data.orientation.z,
                            data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.robot.rz = (yaw + math.pi)*self.factor.rz + self.origin.rz
        if self.robot.rz > math.pi: # Convert the range to -pi to pi
            self.robot.rz = self.robot.rz - 2*math.pi

    """

    def tf_reader(self):
    
        #rospy.init_node('cart_odom_subscriber', anonymous=True)
        rospy.Subscriber("tf", tfMessage, self.tf_callback)

    def tf_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Angular Velocity X: %f", data.transform.rotation.x)
        rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Y: %f", data.transform.rotation.y)
        rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Z: %f", data.transform.rotation.z)
        rospy.loginfo(rospy.get_caller_id() + "Position X: %f", data.transform.translation.x)
        rospy.loginfo(rospy.get_caller_id() + "Position Y: %f", data.transform.translation.y)
        rospy.loginfo(rospy.get_caller_id() + "Position Z: %f", data.transform.translation.z)

    """

    def scan_reader(self):
    
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        
        
    def scan_callback(self, data):

        pass
        #rospy.loginfo("Received a LaserScan message with %d ranges" % len(data.ranges))
        #rospy.loginfo(f"Lidar: {data.ranges[100:260]}")

        
    def drive_controller(self):
    
        self.drive_pub = rospy.Publisher('cart_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.drive_control = Twist();
    	
    	
    # Backend to App
    
    def app_reader(self):
        
        rospy.Subscriber("cart_app2backend", App, self.app_callback)
        
        
    def app_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + f"Message: {data.status}")

        if data.status == "Next Item" or data.status == "Item Path" or data.status == "Destination":

            # Update paths
            self.pathx_app = list(data.pathx)
            self.pathy_app = list(data.pathy)
            # NOTE: the 3D map has the y-axis inverted
            self.pathx_map = [((px + self.map_origin[0]) - self.image_size[0]/2 + 0.5)*self.image_scale for px in data.pathx]
            self.pathy_map = [((-py - self.map_origin[1]) + self.image_size[1]/2 - 0.5)*self.image_scale for py in data.pathy]

            # Data information
            rospy.loginfo(rospy.get_caller_id() + f"Path X: {self.pathx_app}")
            rospy.loginfo(rospy.get_caller_id() + f"Path Y: {self.pathy_app}")
            rospy.loginfo(rospy.get_caller_id() + f"Path X (Map): {self.pathx_map}")
            rospy.loginfo(rospy.get_caller_id() + f"Path Y (Map): {self.pathy_map}")

            # Update robot status
            if data.status == "Item Path":
                self.status = "Stop"
            else:
                self.status = "Continue"

        # Update robot status
        if data.status == "Pause":
            self.status = "Pause"
        elif data.status == "Resume":
            self.status = "Continue"

        #self.app_data = data
      
        
    def app_reporter(self):
    
        self.app_pub = rospy.Publisher('cart_backend2app', Backend, queue_size=10)
    	
    	
    # Backend Cycle
    
    def cycle(self):
    	
        rate = rospy.Rate(2) # 10hz

        while not rospy.is_shutdown():

            #(trans,rot) = self.tflistener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            #print(f"Map location: trans {trans}, rotation {rot}")
            self.tf_update()
            

            # App communication
            bck_msg = Backend()
            bck_msg.status = self.status
            bck_msg.x = (self.robot.px/self.image_scale - 0.5 + self.image_size[0]/2 ) - self.map_origin[0]
            bck_msg.y = (-self.robot.py/self.image_scale - 0.5 + self.image_size[1]/2 ) - self.map_origin[1]

            #print(f"Status: {self.status}")

            # Update robot movement
            if self.status == "Continue":

                # Compute distance and angle of next node
                if self.pathx_map and self.pathy_map:

                    # Direction
                    dirx = self.pathx_map[0] - self.robot.px
                    diry = self.pathy_map[0] - self.robot.py

                    # Angular coordinates
                    distance = math.sqrt(dirx**2 + diry**2)
                    angle = math.atan2(diry, dirx)

                    yaw = angle - self.robot.rz # Correct rz angle
                    if yaw > math.pi: # Convert the range to -pi to pi
                        yaw = yaw - 2*math.pi
                    elif yaw  < -math.pi:
                        yaw = yaw + 2*math.pi

                    # Remove graph node if it close to certain threshold
                    if distance < self.dist_threshold:
                        self.pathx_app.pop(0)
                        self.pathy_app.pop(0)
                        self.pathx_map.pop(0)
                        self.pathy_map.pop(0)

                    # Pid
                    if math.sqrt(yaw*yaw) < math.pi/16:
                        self.drive_control.linear.x = self.P*min(max(distance, 0.2), 1.0) #+ self.I*self.drive_control.linear.x + self.D*(self.linearvel[-1] - self.linearvel[-2])
                    else:
                        self.drive_control.linear.x = 0.0 #+ self.I*self.drive_control.linear.x + self.D*(self.linearvel[-1] - self.linearvel[-2])
                    
                    if math.sqrt(yaw*yaw) > math.pi/16:
                        if yaw > 0.01:
                            self.drive_control.angular.z = 1.2
                        elif yaw < -0.01:
                            self.drive_control.angular.z = -1.2
                        else:
                            self.drive_control.angular.z = 0.0
                        #self.drive_control.angular.z = max(min(self.P*yaw, 1.5), -1.5) #+ self.I*self.drive_control.angular.z + self.D*(self.angularvel[-1] - self.angularvel[-2])
                    else:
                        self.drive_control.angular.z = 0.0

                    print(f"Status: {self.status} | Distance: {distance}, Angle: {angle}, Robot Dir: {self.robot.rz}, Yaw: {yaw} | Linear X: {self.drive_control.linear.x}, Angle Z: {self.drive_control.angular.z}")
                    rospy.loginfo(rospy.get_caller_id() + f"Path X (Map): {self.pathx_map}")
                    rospy.loginfo(rospy.get_caller_id() + f"Path Y (Map): {self.pathy_map}")
                else:
                    self.status = "Destination"
                    print(f"Status: {self.status} | Linear X: {self.drive_control.linear.x}, Angle Z: {self.drive_control.angular.z}")
            else:
                self.drive_control.linear.x = 0.0;
                self.drive_control.angular.z = 0.0;
                print(f"Status: {self.status} | Linear X: {self.drive_control.linear.x}, Angle Z: {self.drive_control.angular.z}")

            # Update velocities
            self.linearvel.append(self.drive_control.linear)
            self.linearvel.pop(0)
            self.angularvel.append(self.drive_control.angular)
            self.angularvel.pop(0)

            # Publish data
            self.drive_pub.publish(self.drive_control)
            self.app_pub.publish(bck_msg)
            rate.sleep()

            #print(f"X: {self.robot.px}, Y: {self.robot.py}, Z: {self.robot.pz} | roll: {self.robot.rx}, pitch: {self.robot.ry}, yaw: {self.robot.rz}")


    def rot_calibration(self):

    	
        rate = rospy.Rate(2) # 10hz
        self.drive_control.angular.z = 1.2

        while not rospy.is_shutdown():

            # App communication
            bck_msg = Backend()
            bck_msg.status = self.status
            bck_msg.x = (self.robot.px/self.image_scale - 0.5 + self.image_size[0]/2 ) - self.map_origin[0]
            bck_msg.y = (-self.robot.py/self.image_scale - 0.5 + self.image_size[1]/2 ) - self.map_origin[1]


            (trans,rot) = self.tflistener.lookupTransform('map', 'base_link', rospy.Time(0))
            print(f"Map location: trans {trans}, rotation {rot}")
            print(f"X: {self.robot.px}, Y: {self.robot.py}, Z: {self.robot.pz} | roll: {self.robot.rx}, pitch: {self.robot.ry}, yaw: {self.robot.rz}")

            if self.robot.rz > math.pi/2:
                self.drive_control.angular.z = 0.0

            # Publish data
            self.drive_pub.publish(self.drive_control)
            self.app_pub.publish(bck_msg)
            rate.sleep()

            
	
		
if __name__ == '__main__':

    cart_bcknd = CartBackend()
    cart_bcknd.cycle()
    #cart_bcknd.rot_calibration()
    

