import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#flask ask
import logging
from flask import Flask,render_template
import json
import requests
import time
import unidecode
from flask_ask import Ask,statement,question,session

## Initialize flask
app= Flask(__name__)
ask= Ask( app , "/apsar_test")

#log= logging.getLogger()
#log.addHandler(logging.StreamHandler())
#log.setLevel(logging.DEBUG)
#logging.getLogger("flask_ask").setLevel(logging.DEBUG)

# ROS pub init
pub = rospy.Publisher('chatter',String,queue_size=10)
rospy.init_node('talker',anonymous=True)
rate= rospy.Rate(10)

#while not rospy.is_shutdown():
@ask.launch
def start_skill():
    welcome_message= 'Hello, would you like to tell me an action?'
    return question(welcome_message)

@ask.intent("NavigateIntent")
def navigate_intent(Place):
    if(Place == 'forward'):
        pub.publish('forward')
        rate.sleep()
        time.sleep(2)
        return statement("moving forward")

    if(Place == 'backward'):
        pub.publish('backward')
        rate.sleep()
        time.sleep(2)
        return statement("moving {}.".format(Place))

    if(Place == 'square'):
        pub.publish('square')
        rate.sleep()
        time.sleep(4)
        return statement("performing square action")

    if(Place == 'stop'):
        pub.publish('stop')
        rate.sleep()
        time.sleep(1)
        return statement("stopping")

@ask.intent("HelpIntent")
def help ():
    return statement ("Available actions are: forward, backward")

@ask.intent('StopIntent')
def stop_intent():
    bye_text='good bye'
    rospy.on_shutdown(bye_text)
    return statement (bye_text)


if __name__ == '__main__':
    app.run(debug=True)





