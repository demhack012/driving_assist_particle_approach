import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
import math

angle = 0
location = 0
obstacle = 0
k_c_x = 0.4
k_c_y = 0.6
k_u = 0.3
k_s_x = 0.3
k_s_y = 0.4

def callback_slope(data):
    global angle
    angle = math.atan(data.data)

def callback_location(data):
    global location
    location = data.data

def callback_ultrasonic(data):
    global obstacle
    obstacle = data.data

def callback_control(data):
    global angle
    global location
    global obstacle
    global k_c_x
    global k_c_y
    global k_s_x
    global k_s_y
    global k_u
    vy = data.data[0] * math.cos(data.data[1]*math.pi/180)
    vx = data.data[1] * math.sin(data.data[1]*math.pi/180)
    if obstacle:
        if location == 2:
            vxf = vx * k_c_x + vx * math.sin(0.349) * k_u + vx * math.sin(angle) * k_s_x
        else:
            vxf = vx * k_c_x - vx * math.sin(0.349) * k_u + vx * math.sin(angle) * k_s_x
    else:
        vxf = vx * k_c_y + vx * math.sin(angle) * k_s_y
    vyf = vy * k_c_y + vy * math.cos(angle)* k_s_y

    vf = math.sqrt(vxf**2 + vyf**2)
    angle_f = math.atan(vxf/vyf)
    angle_degree_f = angle_f * 180/math.pi
    pub_cytron.publish(int(vf))
    pub_servo.publish(int(70 + angle_degree_f))






if __name__ == '__main__':
    try:
        rospy.init_node('combined', anonymous=True)
        rospy.Subscriber('/slope', Float32, callback_slope)
        rospy.Subscriber('/ultrasonic_data', Int8, callback_ultrasonic)
        rospy.Subscriber('/location', Int8, callback_location)
        rospy.Subscriber('/control', Int8MultiArray, callback_control)
        pub_cytron = rospy.Publisher('motor', Int8, queue_size=10)
        pub_servo = rospy.Publisher('servo', Int8, queue_size=10)
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
