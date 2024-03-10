/*
	This file contains the main JavaScript code for the web interface of the robot.
	Much of the basis for this code came from Rui Santos's ESP32 tutorials: https://randomnerdtutorials.com/projects-esp32/
	Modifications made by Collin Schofield, with assistance from Github Copilot.
*/

//=======================================================
// Global variables
//=======================================================

// var botPath = `ws://${window.location.hostname}/ws`;
var botPath = `ws://192.168.137.22:80/ws`;
var websocket;

window.addEventListener('load', onload);// Initialize the websocket when the page is loaded

//=======================================================
// Bot Command Functions
//=======================================================

// Function to send the startBot c	ommand to the ESP32
function startBot() {
	console.log('Sending the startBot command');
	websocket.send(
		JSON.stringify({
			cmd: 'startBot'
		})
	);
}



// Function to send the stopBot command to the ESP32
function stopBot() {
	console.log('Sending the stopBot command');
	websocket.send(
		JSON.stringify({
			cmd: 'stopBot'
		})
	);
}



// Function to toggle if the bot is running or not
function toggleBot() {
	if (document.getElementById('botRunning').innerHTML == 'true') {
		stopBot();
	} else {
		startBot();
	}
}



// Function to send the calibrateSensor command to the ESP32
function calibrateSensor() {
	console.log('Sending the calibrateSensor command');
	websocket.send(
		JSON.stringify({
			cmd: 'calibrateSensor'
		})
	);
}



// Function to get current PID Values
function getPID() {
	console.log('Sending the getPID command');
	websocket.send(
		JSON.stringify({
			cmd: 'getPID'
		})
	);
}



// Function to send new PID values
function updatePID(newKp, newKi, newKd) {
	console.log('Sending the updatePID command, with new PID values: ' + newKp + ', ' + newKi + ', ' + newKd);
	websocket.send(
		JSON.stringify({
			cmd: 'updatePID',
			Kp: newKp,
			Ki: newKi,
			Kd: newKd
		})
	);
	setTimeout(getPID, 1000);// Get the new PID values after 1 second
}



// Sumbit the form to update the PID values. If a value is empty, use the current value
function submitForm() {
	const kpInput = document.getElementById('newTmpKP').value;
	const kiInput = document.getElementById('newTmpKI').value;
	const kdInput = document.getElementById('newTmpKD').value;
	var newKp;
	var newKi;
	var newKd;
	if (kpInput == '') {
		newKp = document.getElementById('kp').innerHTML.value;
	}else{
		newKp = kpInput;
	}
	if (kiInput == '') {
		newKi = document.getElementById('ki').innerHTML.value;
	}else{
		newKi = kiInput;
	}
	if (kdInput == '') {
		newKd = document.getElementById('kd').innerHTML.value;
	}else{
		newKd = kdInput;
	}
	updatePID(newKp, newKi, newKd);
}


// Tell the ESP32 to save the current PID values to the EEPROM
function savePID() {
	console.log('Sending the savePID command');
	websocket.send(
		JSON.stringify({
			cmd: 'savePID'
		})
	);
}



// Get the current max speed of the motor
function getMotorMaxSpeed() {
	console.log('Sending the getMotorMaxSpeed command');
	websocket.send(
		JSON.stringify({
			cmd: 'getMotorMaxSpeed'
		})
	);
}



// Update the motor max speed
function submitMaxSpeedForm() {
	const maxSpeedInput = document.getElementById('newTmpMaxSpeed').value;
	var newMaxSpeedIn;
	if (maxSpeedInput == '') {
		newMaxSpeedIn = document.getElementById('motorAbsMaxSpeed').innerHTML.value;
	}else{
		if (maxSpeedInput > document.getElementById('motorAbsMaxSpeed').innerHTML.value) {
			newMaxSpeedIn = document.getElementById('motorAbsMaxSpeed').innerHTML.value;
		}else{
			newMaxSpeedIn = maxSpeedInput;
		}
	}
	console.log('Sending the updateMaxSpeed command, with new max speed: ' + newMaxSpeedIn);
	websocket.send(
		JSON.stringify({
			cmd: 'updateMotorMaxSpeed',
			newMotorMaxSpeed: newMaxSpeedIn
		})
	);
	setTimeout(getMotorMaxSpeed, 1000);// Get the new max speed after 1 second
}




// Trigger the ESP32 to send the current Sensor Values
function readSensor() {
	console.log('Sending the getSensorValues command');
	websocket.send(
		JSON.stringify({
			cmd: 'readSensor'
		})
	);
}



// Clear the iError value in the PID controller
function clearIError() {
	console.log('Sending the clearIError command');
	websocket.send(
		JSON.stringify({
			cmd: 'cleariError'
		})
	);
}



// Start Run Time Logging
function startRuntimeLogging() {
	console.log('Sending the runTimeLogOn command');
	websocket.send(
		JSON.stringify({
			cmd: 'runTimeLogOn'
		})
	);
}



// Stop Run Time Logging
function stopRuntimeLogging() {
	console.log('Sending the runTimeLogOff command');
	websocket.send(
		JSON.stringify({
			cmd: 'runTimeLogOff'
		})
	);
}



// Toggle Run Time Logging
function toggleRuntimeLogging() {
	if (document.getElementById('logRunTimes').innerHTML == 'true') {
		stopRuntimeLogging();
	} else {
		startRuntimeLogging();
	}
}



// Function to send the getStateInfo command to the ESP32 (Not yet implemented on bot)
function getState() {
	console.log('Sending the getState command');
	websocket.send(
		JSON.stringify({
			cmd: 'getState'
		})
	);
}





//=======================================================
// WebSocket Functions
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
	setTimeout(getState, 1000);
}



// Actions to take when the WebSocket connection is closed
function onClose(event) {
	console.log('Websocket Connection closed');
	setTimeout(initWebSocket, 2000);
}



// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
	try {
		tmp = JSON.parse(event.data);
		if(tmp.message == undefined){
			console.log(tmp);// Print the whole JSON of recieved data in the console, if it is not just a message
		}
	}catch(e){// If the JSON parsing fails, print the error and the data in the console
		console.log("Error parsing JSON: " + e);
		console.log(event.data);
	}
	var receivedObj = JSON.parse(event.data);
	var keys = Object.keys(receivedObj);

	for (var i = 0; i < keys.length; i++){// Loop through all keys in the received object
		var key = keys[i];
		if(key == "message"){// If the key is message, print the message in the console
			console.log(receivedObj[key]);
			continue;
		}else{// Otherwise, update the value of each element in the HTML with the id of the key
			// Check for special keys
			if(key == "consts"){// If the key is consts, update the values of the PID constants in the HTML
				var consts = receivedObj[key];
				for(constKey in consts){
					document.getElementById(constKey).innerHTML = consts[constKey];
				}
				continue;
			}else if(key == "runTimes"){// If the key is runTimes, update the values of the run times in the HTML
				var runTimes = receivedObj[key];
				for(runKey in runTimes){
					document.getElementById(runKey).innerHTML = runTimes[runKey];
				}
				continue;
			}

			try{// Try to update the element with the id of the key
				document.getElementById(key).innerHTML = receivedObj[key];
			}catch(e){
				console.log("Error updating element with id " + key + ": " + e);
			}

			// Call the appropriate function to update the color of the button, based on the key
			if (key === "calibrating") {
				setCalibrateButtonColor(); // Call setCalibrateButtonColor after updating the calibrating element
			} else if (key === "botRunning") {
				setBotRunningButtonColor(); // Call setBotRunningButtonColor after updating the botRunning element
			}
		}
	}
}





//=======================================================
// Other Functions
//=======================================================

// Set the color of the calibrate button, based on if the bot is currently calibrating or not
function setCalibrateButtonColor() {
	if (document.getElementById('calibrating').innerHTML == 'true') {
		document.getElementById('calibrateButton').style.backgroundColor = 'red';
	} else {
		document.getElementById('calibrateButton').style.backgroundColor = 'green';
	}
}



// Set the color of the botRunning button, based on if the bot is currently running or not
function setBotRunningButtonColor() {
	if (document.getElementById('botRunning').innerHTML == 'true') {
		document.getElementById('botRunningButton').style.backgroundColor = 'red';
	} else {
		document.getElementById('botRunningButton').style.backgroundColor = 'green';
	}
}