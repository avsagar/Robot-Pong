#!/usr/bin/env python
import bluetooth
import sys
import numpy as np
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg

def transform_listener():
  rospy.init_node('transform_listener', anonymous=True)

  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(50) # 50hz

      ##### AT START OF GAME ######

  # SEND BALL BOT RANDOM TRAJECTORY
  #random integer values between 0 and 9
  ball_traj =np.array(np.random.randint(9,size=(2,1)))
  #trajectory vector will be converted into a motor command by the ball robot.
  left_line_counter = 0
  right_line_counter = 0
  goal_line_counter = 0

  #br_ball = tf2_ros.TransformBroadcaster()
  #t_ball = geometry_msgs.msg.TransformStamped()
  #br_opp = tf2_ros.TransformBroadcaster()
  #t_opp = geometry_msgs.msg.TransformStamped()

  #t_ball.header.stamp = rospy.Time.now()
  #t_opp.header.stamp = rospy.Time.now()
  #pub = rospy.Publisher('INSTERT TOPIC HERE',Transform, queue_size=10)
  
  #t_ball.header.frame_id = "world"
  #t_ball.child_frame_id = "base_link"
  #t_opp.header.frame_id = "world"
  #t_opp.child_frame_id = base_link
  m1 = 200 #strength of motor1
  m2 = 220 #strength of motor2
  sendCommand(2, m1,m2)
  print("sent command'go forward1'")
  cross = 0
  c = 6
  print 'ball started'
  changed = False
  todo = 2 #1 = stopped, 2 = go, 3 = spin
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    count = 0
    r.sleep()
    try:
      #  world frame is ar_marker_0, ar_ball_frame is ar_marker_1
      trans_ball = tfBuffer.lookup_transform('ar_marker_0', 'ar_marker_2', rospy.Time())
      trans_opp = tfBuffer.lookup_transform('ar_marker_0', 'ar_marker_8', rospy.Time())
      trans_dist_opp = tfBuffer.lookup_transform('ar_marker_8', 'ar_marker_2', rospy.Time())

      #pub.publish(control_command)
      #br_ball.sendTransform(t_ball)
      #br_opp.sendTransform(t_opp)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
      r.sleep()
      continue
    trans_array = [trans_ball.transform.translation.x,trans_ball.transform.translation.y]
    trans_array_op = [trans_opp.transform.translation.x,trans_opp.transform.translation.y]
    dist_opp = [trans_dist_opp.transform.translation.x, trans_dist_opp.transform.translation.y]
    q = [trans_ball.transform.rotation.x,trans_ball.transform.rotation.y,trans_ball.transform.rotation.z,trans_ball.transform.rotation.w]
    q_op = [trans_opp.transform.rotation.x,trans_opp.transform.rotation.y,trans_opp.transform.rotation.z,trans_opp.transform.rotation.w]
    rot_array = tf_conversions.transformations.euler_from_quaternion(q)
    rz=rot_array[2]
    trans_array =np.around(trans_array, decimals=3)
    rot_array_euler = np.around(rz, decimals=3)
    time = rospy.Time()
    # print("At time", time)
    print("Pos x,y (ball): ",trans_array, rz)
    # print('Rotation rz: ', rz)
    # print("Translation x,y (opp): ", dist_opp)

    tol = 2*.0873
    if trans_array[1] < 0.1651: #over front line
      rot = rz > -rz-tol and rz < -rz+tol #is it at desired angle?
      print(rot)
      print(cross)
      if cross >= c and not rot:
        print("ball over front line, reversing direction")
        print("sent command spin1")
        if todo == 1:
          todo = 3
          changed = True
        elif todo == 3:
          changed = False 
        else:
          todo = 1
          changed = True
      elif cross < c:
        cross = cross + 1
      elif (rot or count > 2000):
        if todo == 1:
          todo = 2
          changed = True
        elif todo == 2:
          changed = False
        else:
          todo = 1
          changed = True
        print("sent command go forward2")
        cross = 0
        count = 0
      else:
        count = count + 1
    elif trans_array[1] > 0.889: #over back line
      rot = rz > -rz-tol and rz < -rz+tol
      if cross >= c and not rot:
        print("ball over back line, reversing direction")
        print("sent command spin")
        if todo == 1:
          todo = 3
          changed = True
        elif todo == 3:
          changed = False
        else:
          todo = 1
          changed = True
        count = count + 1
      elif cross < c:
        cross = cross + 1
      elif (rot or count > 2000):
        if todo == 1:
          todo = 2
          changed = True
        elif todo == 2:
          changed = False
        else:
          todo = 1
          changed = True
        count = count + 1
        print("sent command go forward3")
        if todo == 1:
          todo = 2
          changed = True
        elif todo == 2:
          changed = False
        else:
          todo = 1
          changed = True
        count = 0
        cross = 0
      else:
        count = count + 1
    elif trans_array[0] > 1.4224: #over opponent line
      if cross >= c:
        if (dist_opp[1]) > .2032:
          print("ball over goal line, AI wins!")
          print("sent command spin")
          if todo == 1:
            todo = 2
            changed = True
          elif todo == 2:
            changed = False
          else:
            todo = 1
            changed = True
          active = False
          count = 0
        else:
          t_target = 0
          if rz > 0:
            t_target = 3.1415-rz
          else:
            t_target = -3.1415-rz
          rot = rz > t_target-tol and rz < t_target+tol
          if cross >= c and not rot:
            print("ball at AI, reversing direction")
            print("sent command spin")
            if todo == 1:
              todo = 3
              changed = True
            elif todo == 3:
              changed = False
            else:
              todo = 1
              changed = True
          elif rot:
            cross = 0
            if todo == 1:
              todo = 2
              changed = True
            elif todo == 2:
              changed = False
            else:
              todo = 1
              changed = True


      else:
        cross = cross+1
    elif trans_array[0] < .1016: #over left line
      t_target = 0
      if rz > 0:
        t_target = 3.1415-rz
      else:
        t_target = -3.1415-rz
      rot = rz > t_target-tol and rz < t_target+tol
      if cross >= c and not rot:
        if todo == 1:
          todo = 3
          changed = True
        elif todo == 3:
          changed = False
        else:
          todo = 1
          changed = True
        print("ball over left line, reversing direction")
        print("sent command spin")
        cross = 0
      elif cross < c:
        cross = cross + 1
      elif (rot or count > 2000):
        print("sent command go forward4")
        count = 0
        if todo == 1:
          todo = 2
          changed = True
        elif todo == 2:
          changed = False
        else:
          todo = 1
          changed = True
      else:
        count = count + 1
    else:
      # if not sentStraight:
      print todo
      if todo == 1:
        todo = 2
        changed = True
        # sentStraight = True
        # sentSpin = False
      # else:
      #   pass
        #rospy.Timer(rospy.Duration(2), sendCommand(2,m1,m2), oneshot=False)

    ##Opponent Bot controller
    om1 = 200 #motor strength 1
    om2 = 200 #motor strength 2
    todo2 = 1
    changed2 = False
    if dist_opp[1] > 0.1:
      if todo2 == 1:
        todo2 == 2
        changed2 = True
        # print 'opp moving forward'
      elif todo2 == 2:
        print 'opp moving forward'
        changed2 = False
      else:
        todo2 = 1
        changed2 = True
    elif dist_opp[1] < -0.1 and (forward or ind == 0):
      if todo2 == 1:
        todo2 == 3
        changed2 = True
      elif todo2 == 3:
        print 'opp moving backward'
        changed2 = False
      else:
        todo2 = 1
        changed2 = True
    else:
      if todo2 == 2 or todo2 == 3:
        todo2 = 1
        changed2 = True
      else:
        changed2 = False
    ind = ind + 1;


    if changed:
      if todo == 1:
        sendCommand(2,0,0)
        rospy.sleep(.2)
        sendCommand(2,0,0)
      elif todo == 2:
        sendCommand(2,m1,m2)
        rospy.sleep(.2)
        sendCommand(2,m1,m2)
      else:
        sendCommand(2,m1,-m2)
        rospy.sleep(.2)
        sendCommand(2,m1,-m2)
      changed = False

    if changed2:
      if todo2 == 1:
        sendCommand(1,0,0)
        # rospy.sleep(.2)
        # sendCommand(1,0,0)
      elif todo2 == 2:
        sendCommand(1,-om1,-om2)
        # rospy.sleep(.2)
        # sendCommand(1,m1,m2)
      else:
        sendCommand(1,om1,om2)
        # rospy.sleep(.2)
        # sendCommand(1,m1,-m2)
      changed2 = False
    print(changed)
    print(changed2)

def sendCommand(rob, a, b):
  # a = 0
  # b = 0
  if rob == 1:
		sock1.send(str(a)+","+str(b))
  elif rob == 2:
		sock2.send(str(a)+","+str(b))

if __name__ == '__main__':
	# Check if the node has received a signal to shut down
	# If not, run the talker method

	#Run this program as a new node in the ROS computation graph 
	#called /turtlebot_controller.

	#rospy.init_node('transform_publisher', anonymous=True)
	#listener = tf.TransformListener()
	target_address1 = "98:D3:71:FD:51:47"
	port1 = 1
	target_address2 = "00:14:03:06:2D:E1"
	port2 = 2
	sock1 = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
	sock1.connect((target_address2, port1))
	sock2 = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
	sock2.connect((target_address1, port1))
	print 'connected'
	sock1.settimeout(2.0)
	sock2.settimeout(2.0)


	try:
		transform_listener()
	except rospy.ROSInterruptException:
		pass
