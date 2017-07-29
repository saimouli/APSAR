'use strict';
var Alexa = require("alexa-sdk");
var APP_ID = "amzn1.ask.skill.d64bba5d-892f-4d8c-a83d-3bc16ffb58ef"; // get this from website
var EMITTER = require('eventemitter2');
var ROSLIB = require('roslib');

var talkmsg = 'Hello there. Setting up robot.';

var rbServer = new ROSLIB.Ros({
    //url : 'ws://localhost:9090'
    url : 'ws://192.168.0.20:9090' // replace this with the ip adress
 });

 console.log('inside ROSLIB function ip');
 // This function is called upon the rosbridge connection event
 rbServer.on('connection', function() {
     // Write appropriate message to #feedback div when successfully connected to rosbridge
     talkmsg = 'Connected to websocket server.';
     console.log('Connected to websocket server.');
 });

// This function is called when there is an error attempting to connect to rosbridge
rbServer.on('error', function(error) {
    // Write appropriate message to #feedback div upon error when attempting to connect to rosbridge
    talkmsg = 'Error connecting to websocket server.';
    console.log('Error connecting to websocket server: ',error);
});

// This function is called when the connection to rosbridge is closed
rbServer.on('close', function() {
    // Write appropriate message to #feedback div upon closing connection to rosbridge
    talkmsg = 'Connection to websocket server closed.';
    console.log('Connection to websocket server closed.');
 });

// These lines create a topic object as defined by roslibjs
var cmdVelTopic = new ROSLIB.Topic({
    ros : rbServer,
    name : '/turtle1/cmd_vel',
    messageType : 'geometry_msgs/Twist'
});

// These lines create a message that conforms to the structure of the Twist defined in our ROS installation
// It initalizes all properties to zero. They will be set to appropriate values before we publish this message.
var twist = new ROSLIB.Message({
    //console.log('in twist');
    linear : {
        x : 2.0,
        y : 0.0,
        z : 0.0
    },
    angular : {
        x : -2.0,
        y : 0.0,
        z : 0.0
    }
});


function moveTurtle (x1,y1,z1,x2,y2,z2){
    console.log('moveTurtle');
    twist.linear.x= x1;
    twist.linear.y= y1;
    twist.linear.z= z1;

    twist.angular.x= x2;
    twist.angular.y= y2;
    twist.angular.z= z2;

    cmdVelTopic.publish(twist);
}

exports.handler = function(event, context, callback) {
    console.log('Setting up exports.handler for alexa');
    var alexa = Alexa.handler(event, context);
    alexa.appId = APP_ID;
    alexa.registerHandlers(handlers);
    alexa.execute();
};

function functionTalk() {
    // all the stuff you want to happen after that pause
    console.log('pausing in functionTalk');
    //this.emit(':tell', talkmsg)
}

var handlers = {
    'LaunchRequest': function () {
        talkmsg= 'Tell an action';
        this.emit(':tell',talkmsg);
    },
    'TurtlesimIntent': function () {
        var action= this.event.request.intent.slots.Action.value.toLowerCase();

        if (action.includes("forward")){
            console.log('moving forward intent');
            talkmsg = "moving "+action;
            moveTurtle(2.0, 0, 0, 0, 0, 0);
        }

        if (action.includes("backward")){
            console.log('moving backward intent');
            talkmsg = "moving "+action;
            moveTurtle(-2.0, 0, 0, 0, 0, 0);
        }

        if (action.includes("turn right")){
            console.log('moving right intent');
            talkmsg = "Turning"+action;
            moveTurtle(0, 0, 0, 0.785, 0, 0);
        }

        if (action.includes("turn left")){
            console.log('moving right intent');
            talkmsg = "Turning"+action;
            moveTurtle(0, 0, 0, -0.785, 0, 0);
        }

        console.log('Speaking response with delay');
        setTimeout(() => {
            this.emit(':tell', talkmsg);
        }, 500)
        console.log('After speaking response');

    },
};
