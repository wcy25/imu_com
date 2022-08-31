#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# 该程序发布IMU_pub 和odom类型消息
from re import I, L
from unittest.loader import VALID_MODULE_NAME
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import   Twist

import tf2_ros
import tf_conversions
import rospy
import serial
import math
import  sys
from select import select

timeout = rospy.get_param("~key_timeout", 0.01)

ser = serial.Serial("/dev/imu+odom", 4800, timeout=0.1)
sum=0
date=[0,0,0,0,0,0,0,0,0,0,0]

def duqu():
    global sum
    global date

    s=ser.readline().decode("utf-8")
    #加速度
    if sum == 1 and s != 'READ\r\n':
        date[0]=float(s)
        sum=2
    elif sum ==2 and s != 'READ\r\n':
        date[1]=float(s)
        sum=3
    elif sum ==3 and s != 'READ\r\n':
        date[2]=float(s)
        sum=4
    #角速度
    elif sum ==4 and s != 'READ\r\n':
        date[3]=float(s)
        sum=5
    elif sum ==5 and s != 'READ\r\n':
        date[4]=float(s)
        sum=6
    elif sum ==6 and s != 'READ\r\n':
        date[5]=float(s)
        sum=7
    #角度
    elif sum ==7 and s != 'READ\r\n':
        date[6]=float(s)
        sum=8
    elif sum ==8 and s != 'READ\r\n':
        date[7]=float(s)
        sum=9
    elif sum ==9 and s != 'READ\r\n':
        date[8]=float(s)
        sum=10
    #小车行驶路程
    elif sum ==10 and s != 'READ\r\n':
        date[9]=float(s)
        sum=11
    #小车前进方向速度
    elif sum ==11 and s != 'READ\r\n':
        date[10]=float(s)
        sum=0
    if  s == 'READ\r\n'  :
        sum=1

def keycon(cmd_vel):
    print("x:%f",cmd_vel.linear.x)
    print("z:%f",cmd_vel.angular.z)
    ser.write("r")
    if(cmd_vel.angular.z>0):
        ser.write("3".encode('utf-8'))
    elif(cmd_vel.angular.z<0):
        ser.write("4".encode('utf-8'))
    elif(cmd_vel.linear.x>0):
        ser.write("1".encode('utf-8'))
    elif(cmd_vel.linear.x<0):
        ser.write("2".encode('utf-8'))
    else:
        ser.write("0".encode('utf-8'))

#date 0——2 加速度  3——5 角速度  6——8 角度
def IMU():

    rospy.init_node("robot_state_pub")  # 初始化节点
    pub = rospy.Publisher("imu", Imu, queue_size=100)  #IMU话题发布者
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=100)  # 里程计话题发布者
    rate = rospy.Rate(100)  # 发送速度
    send_data = Imu()  # 要发送IMU数据
    send_Od_data = Odometry() #  要发送的Odom数据
    br = tf2_ros.TransformBroadcaster()  # 定义TF变换广播者
    trans = TransformStamped()  # 定义广播者要广播的数据类型

    rospy.Subscriber("/cmd_vel", Twist,keycon)
    #TF坐标
    x = 0
    y = 0
    z = 0

    ax =0 
    ay=0
    az=0

    vthx = 0
    vthy = 0
    vthz = 0

    dx = 0
    th = 0

    Vx = 0  #x轴速度
    Vy = 0  #y轴速度
    Vz = 0  #z轴速度

    thx = 0
    thy = 0


    time_js = 0
    time_now = rospy.Time.now().to_sec()
    print("start******")
    ser.write("g")
    while not rospy.is_shutdown():

        duqu()
        
       # th= 0
        
        # 读取IMU三轴的加速度信息
        ax = date[0]
        ay = date[1]
        az = 0 #date [2]

        vthx = 0#date[3]
        vthy = 0#date[4]                                                     
        vthz = date[5]  # 读取IMU三轴的角速度

        thx = date[6]
        thy = date[7]
        th = date[8] # 读取IMU三轴角度信息

        if th>-90 and th<=0:
            th =th-90
        elif th>-180 and th<=-90:
            th = th+270
        elif th>0 and th<=90:
            th = th-90
        elif th>90 and th<=180:
            th = th-90
        dx = date[9]
        Vx = date[10]/1000


	    # 发布TF变换
        time_last = time_now
        time_now = rospy.Time.now().to_sec()
        delta_t = time_now - time_last  # 计算时间间隔

        delta_x = ( Vx * math.cos(th*(math.pi/180) ))*delta_t
        delta_y = ( Vx * math.sin(th*(math.pi/180) ))*delta_t
        #delta_z =  vz*delta_t
	
        y += round(delta_y,5)
        x += round(delta_x,5)

        #发布IMU消息
        send_data.header.stamp = rospy.Time.now()
        send_data.header.frame_id = "imu_link"
        
        send_data.linear_acceleration.x = ax
        send_data.linear_acceleration.y = ay
        send_data.linear_acceleration.z = az

        send_data.angular_velocity.x = vthx
        send_data.angular_velocity.y = vthy
        send_data.angular_velocity.z = vthz

        send_data.orientation.x = thx
        send_data.orientation.y = thy
        send_data.orientation.z = th

        pub.publish(send_data)

        #发布odom消息
	    # TF transform  odom->base_link
        trans.header.stamp = rospy.Time.now()
        trans.header.frame_id = "odom"
        trans.child_frame_id = "base_footprint"
        trans.transform.translation.x = x
        trans.transform.translation.y = y
        trans.transform.translation.z = 0
        # 从偏航角解算出对应的四元数，只考虑平面运动
                          
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, th*math.pi/180.0)
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]
        
        
	    # 发布里程计信息

        send_Od_data.header.stamp = rospy.Time.now()
        send_Od_data.header.frame_id = "odom"
        send_Od_data.child_frame_id = "base_footprint"
        send_Od_data.pose.pose.position.x = x
        send_Od_data.pose.pose.position.y = y
        send_Od_data.pose.pose.position.z = 0
        send_Od_data.pose.pose.orientation.x=q[0]
        send_Od_data.pose.pose.orientation.y=q[1]
        send_Od_data.pose.pose.orientation.z=q[2]
        send_Od_data.pose.pose.orientation.w=q[3]	
        send_Od_data.twist.twist.linear.x = Vx * math.sin(th*(math.pi/180) )
        send_Od_data.twist.twist.linear.y = Vx * math.cos(th*(math.pi/180) )
        send_Od_data.twist.twist.angular.z = vthz
        
        br.sendTransform(trans)
        odom_pub.publish(send_Od_data)

        print('                         :')
        print(ax)
        print(ay)
        print(az)

        print(vthx)
        print(vthy)
        print(vthz)

        print(thx)
        print(thy)
        print(th)

        print(dx)
        print(Vx)

        #print(math.sin(th*(math.pi/180)))
        #print(math.cos(th*(math.pi/180)))
        #print(delta_x)
        #print(delta_y)
        
        print(x)
        print(y)

        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = ''
        if ch == 'q':
            ser.write("o")
            print("******out")
            exit() 
        rate.sleep()


if __name__ == '__main__':
    try:
        IMU()
        
    except rospy.ROSInterruptException:
        print("////////////////////////////////////////////////////////////////////")
        pass
