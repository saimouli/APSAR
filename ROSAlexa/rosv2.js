// Version for sending a goal... working

'use strict';
var Alexa = require("alexa-sdk");
var APP_ID = "amzn1.ask.skill.1914baf4-6ddc-4fdf-a238-4b4a80d7c027";
var EMITTER = require('eventemitter2');
var ROSLIB = require('roslib');

var talkmsg = 'Hello there. Setting up robot Tuesday in the lab';

var ros = new ROSLIB.Ros({
	url : 'ws://123.456.789.10:9090'
});

ros.on('connection', function() {
    talkmsg = 'Connected to websocket server.';
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    talkmsg = 'Error connecting to websocket server';
    console.log('Error connecting to websocket server: ');
	//console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    talkmsg = 'Connection to websocket server closed on Tuesday.';
    console.log('Connection to websocket server closed on Tuesday.');
});


var moveClient = new ROSLIB.ActionClient({
    ros : ros,
    serverName : '/move_base',
    actionName : '/MoveBaseAction'
});

var goal = new ROSLIB.Goal({
    actionClient : moveClient,
    goalMessage: {
         target_pose: {
           header: {
             frame_id: "map"
           },
           pose: {
             position: {
               x: -4.1,
               y: -21.5,
               z: 0.0
             },
             orientation: {
               x: 0.0,
               y: 0.0,
               z: -0.79,
               w: 0.60
             }
           }
         }
      }
});


// This is the attempted function for creating goals
function createGoal(x1, y1, z1, x2, y2, z2, w2) {
	console.log('in createGoal');
	goal = new ROSLIB.Goal({
		actionClient : moveClient,
		goalMessage: {
			 target_pose: {
			   header: {
				 frame_id: "map"
			   },
			   pose: {
				 position: {
				   x: x1,
				   y: y1,
				   z: z1
				 },
				 orientation: {
				   x: x2,
				   y: y2,
				   z: z2,
				   w: w2
				 }
			   }
			 }
		  }
	});
}


goal.on('feedback', function(feedback) {
    talkmsg = 'Goal on feedback';
    console.log('Feedback: ' + feedback.sequence);
});

goal.on('result', function(result) {
    talkmsg = 'Goal on result';
    console.log('Final Result: ' + result.sequence);
});

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
        console.log('In Launch Request');
		talkmsg = 'Where do you want to go?';
		this.emit(':tell', talkmsg);
    },
    'NavigateIntent': function () {
	
        console.log('In NavigateIntent');
		var destination = this.event.request.intent.slots.Place.value;
		destination = destination.toLowerCase();		// make sure it is in all lower case
		if (destination.includes("brandon")) { //This is currently not working. The function must be set up for this to work. 
			console.log('Setting goal to Brandon office');
			talkmsg = "Navigating to Brandon's Office because you said "+destination;
			createGoal(2.22, -19.8, 0.0, 0.0, 0.0, 0.00, 1.00);
		}
		if (destination.includes("tom")) {
			console.log('Setting goal to Tom office');
			talkmsg = "Navigating to Tom's Office because you said "+destination;
			createGoal(-0.5, 3.0, 0.0, 0.0, 0.0, -0.79, 0.60);

		}
		if (destination.includes("hall")) {
			console.log('Setting goal to Hallway');
			talkmsg = "Navigating to the hallway because you said "+destination;
			createGoal(-2.716, -26.783, 0.0, 0.0, 0.0, -0.79, 0.60);

		}
		console.log('In NavigateIntent destination set to '+destination);
		
        goal.send();
        // call the rest of the code and have it execute after 1/2 second
        console.log('Speaking response with delay');
        setTimeout(() => {
            this.emit(':tell', talkmsg);
        }, 500)
        console.log('After speaking response');
    },

};
