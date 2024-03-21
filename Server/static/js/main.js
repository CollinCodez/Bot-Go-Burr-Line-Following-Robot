/*
	This file contains the main JavaScript code for the web interface of the robot.
	Much of the basis for this code came from Rui Santos's ESP32 tutorials: https://randomnerdtutorials.com/projects-esp32/
	Modifications made by Collin Schofield, with assistance from Github Copilot.
*/

//=======================================================
// Global variables
//=======================================================

// var botPath = `ws://${window.location.hostname}/ws`;
var botPath = `ws://192.168.1.5:80/ws`;
var websocket;

window.addEventListener('load', onload);// Initialize the websocket when the page is loaded

const EDFState = Object.freeze({// Enum for the EDF State
	UNINITIALIZED: 0,
	STOPPED: 1,
	RUNNING: 2
});


const outChartEnum = Object.freeze({// Enum for the series's of the output chart
	pOut: 0,
	p2Out: 1,
	dOut: 2,
	iOut: 3,
	out: 4
});


var sensorVals = [];// Array to store the sensor values for the chart
// Charts for the sensor values, error values, and output values
var sensorsChart;
var errorChart;
var outputChart;


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
function updatePID(newKp,newKp2, newKi, newKd) {
	console.log('Sending the updatePID command, with new PID values: ' + newKp + ', ' + newKp2 + ', ' + newKi + ', ' + newKd);
	websocket.send(
		JSON.stringify({
			cmd: 'updatePID',
			Kp: newKp,
			Kp2: newKp2,
			Ki: newKi,
			Kd: newKd
		})
	);
	setTimeout(getPID, 1000);// Get the new PID values after 1 second
}



// Sumbit the form to update the PID values. If a value is empty, use the current value
function submitForm() {
	const kpInput = document.getElementById('newTmpKP').value;
	const kp2Input = document.getElementById('newTmpKP2').value;
	const kiInput = document.getElementById('newTmpKI').value;
	const kdInput = document.getElementById('newTmpKD').value;
	var newKp;
	var newKp2
	var newKi;
	var newKd;
	if (kpInput == '') {
		newKp = document.getElementById('kp').innerHTML.value;
	}else{
		newKp = kpInput;
	}
	if (kp2Input == '') {
		newKp2 = document.getElementById('kp2').innerHTML.value;
	}else{
		newKp2 = kp2Input;
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
	updatePID(newKp,newKp2, newKi, newKd);
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
	console.log('Sending the getSensorVals command');
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



// Function to send the initEDF command to the ESP32
function initEDF() {
	console.log('Sending the initEDF command');
	websocket.send(
		JSON.stringify({
			cmd: 'initEDF'
		})
	);
}



// Function to send the startEDF command to the ESP32
function startEDF() {
	console.log('Sending the startEDF command');
	websocket.send(
		JSON.stringify({
			cmd: 'startEDF'
		})
	);
}



// Function to send the stopEDF command to the ESP32
function stopEDF() {
	console.log('Sending the stopEDF command');
	websocket.send(
		JSON.stringify({
			cmd: 'stopEDF'
		})
	);
}



// Function to toggle between EDF states. If the EDF is running, stop it. If it is stopped, start it. If it is uninitialized, initialize it.
function toggleEDF() {
	if (document.getElementById('edfState').innerHTML == EDFState.UNINITIALIZED) {
		initEDF();
	} else if (document.getElementById('edfState').innerHTML == EDFState.RUNNING) {
		stopEDF();
	} else if (document.getElementById('edfState').innerHTML == EDFState.STOPPED) {
		startEDF();
	}else{
		console.log('Error: EDF state is not valid');
		initEDF();
	}
}



// function to update the EDF speed
function submitEDFForm() {
	const edfSpeedInput = document.getElementById('newTmpEDFSpeed').value;
	var newEDFSpeed;
	if (edfSpeedInput == '') {
		newEDFSpeed = document.getElementById('edfSpeed').innerHTML.value;
	}else if (edfSpeedInput < document.getElementById('edfMinSpeed').innerHTML.value) {
		newEDFSpeed = document.getElementById('edfMinSpeed').innerHTML.value;
	}else if (edfSpeedInput > document.getElementById('edfAbsMaxSpeed').innerHTML.value) {
		newEDFSpeed = document.getElementById('edfAbsMaxSpeed').innerHTML.value;
	}else {
		newEDFSpeed = edfSpeedInput;
	}
	console.log('Sending the updateEDFSpeed command, with new EDF speed: ' + newEDFSpeed);
	websocket.send(
		JSON.stringify({
			cmd: 'setEDFSpeed',
			newEDFSpeed: newEDFSpeed
		})
	);
	// setTimeout(getEDFInfo, 1000);// Get the new EDF speed after 1 second
}



// Function to get the current EDF speed and state
function getEDFInfo() {
	console.log('Sending the getEDFInfo command');
	websocket.send(
		JSON.stringify({
			cmd: 'getEDFInfo'
		})
	);
}





//=======================================================
// WebSocket Functions
//=======================================================

function onload(event) {
	initWebSocket();
	setTimeout(prepCharts, 1000);
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
	document.getElementsByClassName('topnav')[0].style.backgroundColor = 'green';
	setTimeout(getState, 1000);
}



// Actions to take when the WebSocket connection is closed
function onClose(event) {
	console.log('Websocket Connection closed');
	setTimeout(initWebSocket, 2000);
	document.getElementsByClassName('topnav')[0].style.backgroundColor = 'red';
}



// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
	var curTime = (new Date()).getTime();// Save the current time, to be used in the charts

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
			}else if(key == "sensorVals"){// If the key is sensorVals, update the values of the sensor values in the JavaScript array
				sensorVals = receivedObj[key];
				sensorsChart.series[0].setData(sensorVals);
				continue;
			}else if(key == "outs"){// If the key is outs, update the values of the outputs in the charts
				var outs = receivedObj[key];
				
				// Add the new point to the Error chart
				if(errorChart.series[0].data.length > 15){// If the error chart has more than 100 points, remove the first point
					errorChart.series[0].addPoint([curTime, outs.error], true, true, true);
				}else{// Otherwise, just add the new point
					errorChart.series[0].addPoint([curTime, outs.error], true, false, true);
				}

				// Add the new points to the Output chart
				if (outputChart.series[0].data.length > 15) { // If the output chart has more than 100 points, remove the first point
					outputChart.series[outChartEnum.pOut].addPoint([curTime, outs.pOut], false, true, true);
					outputChart.series[outChartEnum.p2Out].addPoint([curTime, outs.p2Out], false, true, true);
					outputChart.series[outChartEnum.dOut].addPoint([curTime, outs.dOut], false, true, true);
					outputChart.series[outChartEnum.iOut].addPoint([curTime, outs.iOut], false, true, true);
					outputChart.series[outChartEnum.out].addPoint([curTime, outs.out], true, true, true);
				} else { // Otherwise, just add the new points
					outputChart.series[outChartEnum.pOut].addPoint([curTime, outs.pOut], false, false, true);
					outputChart.series[outChartEnum.p2Out].addPoint([curTime, outs.p2Out], false, false, true);
					outputChart.series[outChartEnum.dOut].addPoint([curTime, outs.dOut], false, false, true);
					outputChart.series[outChartEnum.iOut].addPoint([curTime, outs.iOut], false, false, true);
					outputChart.series[outChartEnum.out].addPoint([curTime, outs.out], true, false, true);
				}

				continue;
			}// End of special keys

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
			} else if (key === "edfState") {
				setEDFButtonColor(); // Call setEDFButtonColor after updating the edfState element
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



// Set the color of the edf button based on the state of the EDF
function setEDFButtonColor() {
	if (document.getElementById('edfState').innerHTML == EDFState.UNINITIALIZED) {
		document.getElementById('edfButton').style.backgroundColor = 'yellow';
	}else if (document.getElementById('edfState').innerHTML == EDFState.RUNNING) {
		document.getElementById('edfButton').style.backgroundColor = 'red';
	} else {
		document.getElementById('edfButton').style.backgroundColor = 'green';
	}
}



function prepCharts(){
	sensorsChart = new Highcharts.Chart({
		chart: {
			renderTo: 'chart-sensors',
			type: 'spline',
			animation: false
		},
		title: { text: 'Sensor Values' },
		xAxis:{
			categories: [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]
		},
		yAxis: {
			title: {
				text: 'Read Time'
			},
			labels: {
				format: '{value} us'
			},
			min: 0,
			max: 1000
		},
		plotOptions: {
			spline: {
				marker: {
					radius: 4,
					lineColor: '#666666',
					lineWidth: 1
				}
			},
		},
		series: [{
			name: 'Sensor Readings',
			marker: {
				symbol: 'diamond'
			},
			data: sensorVals
		}]
	});

	errorChart = new Highcharts.Chart({
		chart: {
			renderTo: 'chart-error',
			animation: false
		},
		title: { text: 'Error Values' },
		series: [{
			showInLegend: false,
			data: []
		}],
		xAxis: { type: 'datetime',
			dateTimeLabelFormats: { second: '%H:%M:%S' }
		},
		yAxis: {
			title: {
				text: 'Error'
			},
			labels: {
				format: '{value}'
			},
			min: -7000,
			max: 7000
		},
		plotOptions: {
			line: { animation: false,
				dataLabels: { enabled: true }
			},
			series: { color: '#059e8a' }
		},
		series: [{
			name: 'Error Values',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}],
		credits: { enabled: false }
	});


	outputChart = new Highcharts.Chart({
		chart: {
			renderTo: 'chart-output',
			animation: false,
			height: "75%"
		},
		title: { text: 'Output Values' },
		xAxis: { type: 'datetime',
			dateTimeLabelFormats: { second: '%H:%M:%S' }
		},
		yAxis: {
			title: {
				text: 'Output'
			},
			labels: {
				format: '{value}'
			},
			min: -1023,
			max: 1023
		},
		plotOptions: {
			line: { animation: false,
				dataLabels: { enabled: true }
			},
			// series: { color: '#059e8a' }
		},
		series: [{
			name: 'pOut',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}, {
			name: 'p2Out',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}, {
			name: 'dOut',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}, {
			name: 'iOut',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}, {
			name: 'out',
			marker: {
				symbol: 'diamond'
			},
			data: []
		}],
		credits: { enabled: false }
	});
}