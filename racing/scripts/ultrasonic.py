import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int8

# GPIO pin setup
TRIG_PIN = 36
ECHO_PIN = 38
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# ROS publisher
pub = rospy.Publisher('ultrasonic_data', Int8, queue_size=10)
publ = rospy.Publisher('location', Int8, queue_size=10)

location = 1
detected = 0

def measure_distance():
   """Measures the distance using the ultrasonic sensor."""
   GPIO.output(TRIG_PIN, True)
   rospy.sleep(0.00001)  # Delay 10 microseconds
   GPIO.output(TRIG_PIN, False)

   start_time = rospy.get_time()
   stop_time = start_time

   while GPIO.input(ECHO_PIN) == 0:
       start_time = rospy.get_time()

   while GPIO.input(ECHO_PIN) == 1:
       stop_time = rospy.get_time()

   time_elapsed = stop_time - start_time
   distance = (time_elapsed * 34300) / 2  # Speed of sound in cm/s
   return distance

def publish_data():
    global location
    global detected
    """Publishes the distance data to ROS topic."""
    distance = measure_distance()
    print(distance)
    if distance < 50:
        if detected == 0:
            detected = 1
            print("detected")
            pub.publish(1)
            if location == 1:
                location = 2
                print("location 2")
                publ.publish(2)
            else:
                print("location 1")
                location = 1
                publ.publish(1)
    else:
        detected = 0
        print("not detected")
        pub.publish(0)

if __name__ == '__main__':
   rospy.init_node('ultrasonic_sensor')
   rate = rospy.Rate(10)  # Publish at 10 Hz

   while not rospy.is_shutdown():
       publish_data()
       rate.sleep()

   GPIO.cleanup()
