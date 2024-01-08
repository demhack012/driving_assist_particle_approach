# Import libraries
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int8
import rospy


def callback(data):
    # angle = float(input('Enter angle between 0 & 180: '))
    servo1.ChangeDutyCycle(2+(data.data/18))
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)

if __name__ == '__main__':
    try:
        GPIO.setmode(GPIO.BOARD)

        # Set pin 11 as an output, and define as servo1 as PWM pin
        GPIO.setup(11,GPIO.OUT)
        servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz

        # Start PWM running, with value of 0 (pulse off)
        servo1.start(0)# set pwm for M1
        rospy.init_node('servo', anonymous=True)
        rospy.Subscriber('/servo', Int8, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        servo1.stop()
        GPIO.cleanup()
        print("Goodbye!")


