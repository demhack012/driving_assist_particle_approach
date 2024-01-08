import RPi.GPIO as GPIO			# using Rpi.GPIO module
from time import sleep			# import function sleep for delay
from std_msgs.msg import Int8
import rospy

def callback(data):
    p1.start(data.data)

if __name__ == '__main__':
    try:
        GPIO.setmode(GPIO.BOARD)			# GPIO numbering
        GPIO.setwarnings(False)			# enable warning from GPI
        pwm = 15
        dir = 13
        GPIO.setup(pwm, GPIO.OUT)
        GPIO.setup(dir, GPIO.OUT)
        p1 = GPIO.PWM(pwm, 100)			# set pwm for M1
        GPIO.output(dir, GPIO.HIGH)
        p1.start(0)
        rospy.init_node('cytron', anonymous=True)
        rospy.Subscriber('/motor', Int8, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        p1.start(0)
