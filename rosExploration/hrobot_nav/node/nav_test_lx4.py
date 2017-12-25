#!/usr/bin/env python
#coding=utf-8

import rospy
import actionlib
import string
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import String
from std_msgs.msg import Int32

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res
    

class NavTest():
#####################################################################
    
    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))    

    def callbackXFM(self, msg):
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')        
                        
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)  
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")            
        
        # Set the movement command to a rotation
        move_cmd.angular.z = 0.15
            
        # Track the last angle measured
        goal_angle = msg
#        Angle = msg
        if goal_angle>=180:
            last_angle=360-last_angle
            
        # Track how far we have turned
        turn_angle = 0
            
        # Begin the rotation
        while abs(turn_angle) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)                
            r.sleep()
                
            # Get the current rotation
            (position, rotation) = self.get_odom()
                
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)
                
            turn_angle += delta_angle
            last_angle = rotation

        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1.0)
            
#        self.cmd_vel.publish(Twist())             
###############################################################
                
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 2)
        
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        locations = dict()		
        
        locations['A'] = Pose(Point(1.279, -0.265, 0.000), Quaternion(0.000, 0.000, 0.318, 0.948))
        locations['厨房'] = Pose(Point(2.539, 1.657, 0.000), Quaternion(00.000, 0.000, 0.000, 1.000))
        locations['卧室'] = Pose(Point(1.337, 1.090, 0.000), Quaternion(0.000, 0.000, 0.924, 0.382))
        locations['D'] = Pose(Point(-0.324, -0.276, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))      
 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        self.Dist = 0
        self.Asr = ""
        Loc_1=0
        Loc_2=0
        Name_1=0
        name_2=0
        Name=""
        Drink_1=0
        Drink_2=0
        Drink_3=0
        Drink=""
        
        # Get the initial pose from the user (how to set)
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()

        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
                
        dict_keys = ['A','厨房','D','客厅']	
            
        pub1 = rospy.Publisher('/voice/xf_tts_topic', String, queue_size=5)
        pub2 = rospy.Publisher('/voice/xf_asr_topic', Int32, queue_size=5)
        pub3 = rospy.Publisher('arm',String,queue_size=5)
        pub4 = rospy.Publisher('tracking',Int32,queue_size=5)
        pub5 = rospy.Publisher('/following',Int32,queue_size=5)
        pub6 = rospy.Publisher('drink_type',String,queue_size=5)
        pub7 = rospy.Publisher('ros2_wake',Int32,queue_size=5)
        #######################################
        
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            if i == n_locations:
                i = 0
#                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if dict_keys[0] == last_location:
                    i = 1
            
            #####################################
            ####### Modify by XF 2017.4.14 ######
#            location = sequence[i]    
#           rospy.loginfo("location= " + str(location))
#            location = locations.keys()[i]
	    location = dict_keys[i]
            #####################################    
                    
            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # Store the last location for distance calculations
            last_location = location
            
            # Increment the counters
            i += 1
            n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))       
            
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 6 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(360)) 

            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                
                if state == GoalStatus.ABORTED:			# can not find a plan,give feedback
                    rospy.loginfo("please get out")
                    status_n = "please get out"			
                    pub1.publish(status_n)
                    
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                    
                    ############ add voice by XF 4.25 ##################	The 1st
#                    pub1 = rospy.Publisher('/voice/xf_tts_topic', String, queue_size=5)
                    loc ="我已到达"+str(location)
          	    pub1.publish(loc)
          	    
          	    rospy.sleep(5)
          	    
                    rospy.Subscriber('dist',Int32,self.callbackDist)	#now robot already adjust correctly
                    rospy.loginfo("Dist:"+str(self.Dist))

          	    if str(location) == 'A':
          	    	commandA1="您好，有什么可以帮助您?"
          	    	pub1.publish(commandA1)
          	    	          	    	
          	    ############# now next asr ######################	
          	    	awake=1
          	   	pub2.publish(awake)
          	   	rospy.Subscriber('/voice/xf_asr_follow',String,self.callbackAsr)
          	   	###### Modify by XF 2017. 4.21 #########	                
	                rospy.sleep(15)	
	                rospy.loginfo("self.Asr:"+str(self.Asr))
	                
	                Loc_1=string.find(self.Asr,'卧室')
	                Loc_2=string.find(self.Asr,'客厅')   
	                Name_1=string.find(self.Asr,'张三') 
	                Name_2=string.find(self.Asr,'李四')
	                Drink_1=string.find(self.Asr,'可乐')
	                Drink_2=string.find(self.Asr,'雪碧')
	                Drink_3=string.find(self.Asr,'橙汁')
	                if Loc_1!=-1:  
	                    rospy.loginfo("匹配卧室")
                            dict_keys = ['A','厨房','卧室','D']
                        if Loc_2!=-1:
                            rospy.loginfo("匹配客厅")
                            dict_keys = ['A','厨房','客厅','D']
                        if Name_1!=-1:
                            Name = '张三'
                            rospy.loginfo("匹配张三")
                        if Name_2!=-1:
                            Name = '李四'
                            rospy.loginfo("匹配李四")
                        if Drink_1!=-1:
                            Drink = '可乐'
                        if Drink_2!=-1:
                            Drink = '雪碧'
                        if Drink_3!=-1:
                            Drink = '橙汁'
                        
                        ros2_wake=1
                        pub7.publish(ros2_wake)
                        pub6.publish(Drink)
                        #######################################                        
          	   	rospy.sleep(5)
          	    	commandA2="我已获取任务，请稍等."
          	    	pub1.publish(commandA2)          	    
                    ################################################## 	The 2nd               
                    if str(location) == '厨房':
                    	############## add by XF 5.27 ##################
                    	rospy.sleep(5)
                    	track=1
                    	pub4.publish(track)
                    	rospy.sleep(10) 
                    	rospy.Subscriber('dist',Int32,self.callbackDist)	#now robot already adjust correctly
                    	rospy.loginfo("Dist2:"+str(self.Dist))
		        rospy.loginfo("go away Dist:"+str(self.Dist/1000.0-0.3))
                    	self.goal = MoveBaseGoal()
            		self.goal.target_pose.pose.position.x = self.Dist/1000.0-0.3
            		self.goal.target_pose.pose.orientation.w = 1.0; 
            		self.goal.target_pose.header.frame_id = 'base_footprint'
            		self.goal.target_pose.header.stamp = rospy.Time.now()            
		        self.move_base.send_goal(self.goal)		        		        
		        rospy.sleep(5)
                    	################################################                    	
                    	commandB1="pick"
                    	pub3.publish(commandB1)
                    	rospy.sleep(10)
                    	commandB2="我已找到"+str(Drink)+"并抓取."
                    	pub1.publish(commandB2)
                    ##################################################  The 3rd              
                    if str(location) == '卧室':
#                    	rospy.sleep(5)
                        follow=1
                        pub5.publish(follow)
                        
                        track=2			#stop tracking
                    	pub4.publish(track)
                    	rospy.loginfo("send stop tracking command")
                    	
                    	#subscribe the angle
#                    	rospy.Subscriber('xf_xfm_node', Int32, self.callbackXFM)
                    	rospy.sleep(10)                    	
                    	commandC2=str(Name)+"给您"+str(Drink)
                    	pub1.publish(commandC2)
                    	rospy.sleep(3)
                    	commandC1="release"
                    	pub3.publish(commandC1)
                    	rospy.sleep(50)
                    ##################################################                             
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            
            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            
            rospy.sleep(self.rest_time)
         
    def callbackDist(self,msg):
    	self.Dist=msg.data
    
    def callbackAsr(self,msg):
    	self.Asr=msg.data
    	rospy.loginfo("callback:"+str(self.Asr))
    
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

