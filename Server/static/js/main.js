/*
	This file contains the main JavaScript code for the web interface of the robot.
	Much of the basis for this code came from Rui Santos's ESP32 tutorials: https://randomnerdtutorials.com/projects-esp32/
	Modifications made by Collin Schofield, with assistance from Github Copilot.
*/

//=======================================================
// Global variables
//=======================================================

// var botPath = `ws://${window.location.hostname}/ws`;
var botPath = `ws://192.168.127.2:80/ws`;
var websocket;

window.addEventListener('load', onload);// Initialize the websocket when the page is loaded

//=======================================================
// Bot Command Functions
//=======================================================

// Function to send the startBot command to the ESP32
function startBot() {
	console.log('Sending the startBot command');
	websocket.send('startBot');
}



// Function to send the stopBot command to the ESP32
function stopBot() {
	console.log('Sending the stopBot command');
	websocket.send('stopBot');
}



// Function to send the calibrateSensor command to the ESP32
function calibrateSensor() {
	console.log('Sending the calibrateSensor command');
	websocket.send('calibrateSensor');
}



// Function to send the getStateInfo command to the ESP32 (Not yet implemented on bot)
// function getStateInfo() {
//	 console.log('Sending the getStateInfo command');
//	 websocket.send('getStateInfo');
// }



//=======================================================
// Other Functions
//=======================================================

function onload(event) {
	initWebSocket();
}



// Function to initialize the WebSocket connection
function initWebSocket() {
	console.log('Trying to open a WebSocket connection…');
	websocket = new WebSocket(botPath);
	websocket.onopen = onOpen;
	websocket.onclose = onClose;
	websocket.onmessage = onMessage;
}



// Actions to take when the WebSocket connection is opened
function onOpen(event) {
	console.log('Websocket Connection opened');
	// getStateInfo();
}



// Actions to take when the WebSocket connection is closed
function onClose(event) {
	console.log('Websocket Connection closed');
	setTimeout(initWebSocket, 2000);
}



// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
	console.log(event.data);// Print the recieved data in the console
	var receivedObj = JSON.parse(event.data);
	var keys = Object.keys(receivedObj);

	for (var i = 0; i < keys.length; i++){// Loop through all keys in the received object
		var key = keys[i];
		if(key == "message"){// If the key is message, print the message in the console
			console.log(receivedObj[key]);
			continue;
		}else{// Otherwise, update the value of each element in the HTML with the id of the key
			document.getElementById(key).innerHTML = receivedObj[key];
		}
	}
}