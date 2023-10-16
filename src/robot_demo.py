#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32MultiArray, Bool
import time

class small_demo:
    def __init__(self):
        rospy.init_node('robot_demo')
        rospy.Subscriber('/battery_info', Float32MultiArray, self.battery_info_callback)
        rospy.Subscriber('/environmental_info', Float32MultiArray, self.environmental_info_callback)
        rospy.Subscriber('/column_button', Bool, self.button_callback)
        self.pub=rospy.Publisher("/feedback_to_tablet", String, queue_size=10)
        self.battery_percentage = 0
        self.temperature = 25
        self.button_state = False
        self.demo_flag = False

    def environmental_info_callback(self, msg):
        self.temperature = msg.data[5]

    # Callback function to handle incoming messages
    def button_callback(self, msg):
        self.button_state = msg.data

    def speak(self,msg):
        screen_msg = msg
        self.pub.publish("hello,"+ screen_msg+",5")
        #rospy.sleep(14) # screen time + 9

    def demo(self):
        #print(self.button_state,self.demo_flag )
        if self.button_state == True and self.demo_flag == False:
            self.demo_flag == True
            print("demo started")

            self.speak("hello! lovely humans! I hope you're all ready for a dazzling demonstration today! shall we start ?")
            rospy.sleep(15)

            self.speak("Check out my fabulous LEDs! I can change them to diffrent colors ")
            rospy.sleep(5)
            rospy.set_param('/led_2_set', "green")
            rospy.sleep(2)
            rospy.set_param('/led_1_set', "green")
            rospy.sleep(1)
            rospy.set_param('/led_1_set', "off")
            rospy.set_param('/led_2_set', "off")
            rospy.sleep(1)
            self.speak("Red the color of passion and energy! but we use this to show warnings and errors")
            rospy.sleep(5)
            rospy.set_param('/led_1_set', "red")
            rospy.set_param('/led_2_set', "red")
            rospy.sleep(4)
            self.speak("Ah green my favorite color! It's so eco-friendly")
            rospy.sleep(5)
            rospy.set_param('/led_1_set', "green")
            rospy.set_param('/led_2_set', "green")
            rospy.sleep(1)
            rospy.set_param('/led_1_set', "red")
            rospy.sleep(1)
            rospy.set_param('/led_2_set', "red")
            rospy.sleep(1)
            rospy.set_param('/led_1_set', "green")
            rospy.sleep(1)
            rospy.set_param('/led_2_set', "green")

            self.speak(" And speaking of talent I can even tilt my head like this... ")
            rospy.sleep(6)
            rospy.set_param('/neck_cmd', "5")
            rospy.sleep(6)
            self.speak(" or pan it like that! Impressive right?")
            rospy.sleep(4)
            rospy.set_param('/neck_cmd', "6")
            rospy.sleep(6)
            rospy.set_param('/neck_cmd', "2")

            self.speak("Well my dear humans our time together is coming to an end. I hope you had as much fun as I did!")
            rospy.sleep(8)
            self.speak("Until next time! stay awesome!")
            rospy.sleep(2)

            self.demo_flag == False # ready for another demo
            print("ready for new demo")
            print("------------------")

    def battery_info_callback(self, msg):
        self.battery_percentage = msg.data[3]


autonav_node = small_demo()
while not rospy.is_shutdown():
    autonav_node.demo()
    rospy.sleep(0.1)
