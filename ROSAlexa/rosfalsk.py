import logging
from flask import Flask
import json
import requests
import unidecode
from flask_ask import Ask, statement, question, session

# ROS
import rospy
from geometry_msgs.msg import Twist

## Code starts here 
app= Flask(__name__)
ask = Ask(app,"/turtle_sim")

log= logging.getLogger()
log.addHandler(logging.StreamHandler())
log.setLevel(logging.DEBUG)
logging.getLogger("flask_ask").setLevel(logging.DEBUG)

## Setting up ROS publihser 
rospy.init_node('alexatest', anonymous=True)
velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg= Twist()

@ask.launch
def start_skill():
	welcome_message= 'Hello there, please mention an action'
	return question(welcome_message)

@ask.intent("NavigateIntent")
def navigate_intent(Place):
	speed =2.0
	if (Place == 'forward'):
		vel_msg.linear.x= speed
		velocity_publisher.publish(vel_msg)
		return statement("moved {}.".format(Place))

	if (Place == 'backward'):
		vel_msg.linear.x= -speed
		velocity_publisher.publish(vel_msg)	
		return statement("moved {}.".format(Place))

	if (Place == 'turn right' or 'turnright'):
		vel_msg.angular.x = speed
		velocity_publisher.publish(vel_msg)
		return statement("turned right")

	if (Place == 'turn left' or 'turnleft'):
		vel_msg.angular.x= -speed
		velocity_publisher.publish(vel_msg)
		return statement("turned left")

@ask.intent("HelpIntent")
def help ():
	return statement ("Available actions are: forward, backward, turn right and turn left")




if __name__ == '__main__':
	app.run(debug=True)
