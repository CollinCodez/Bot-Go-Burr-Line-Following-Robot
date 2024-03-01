/*
	This Code is made for the "BotGoBrrr" Group (Group 2) Line Following Robot.
	Code was wrote in majority by Collin Schofield, with assistance from Github Copilot.
	Also, much of the basis for the networking code came from Rui Santos's ESP32 tutorials: https://randomnerdtutorials.com/projects-esp32/
*/

// #define CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS	1

#include <Arduino.h>
#include <WiFi.h>
// #include <AsyncTCP.h>// Included in the ESPAsyncWebServer library
#include <ESPAsyncWebServer.h>// This will automatically use whatever core is available, at priority 3
// #include <HTTPClient.h>
// #include <WebServer.h>
// #include <InfluxDbClient.h>
#include <ArduinoJson.h>
#include <QTRSensors.h>




//======================================================================================================
//	Hardware Connections
//======================================================================================================
// GPIO 6-11 are for the flash interface. They seem to be connected to flash chip and should probably be avoided

// Battery Analog Reading Pins
const int mainBatPin = 36;// GPIO36 - ADC 0
const int fanBatPin = 39;// GPIO39 - ADC 3

// Motor Pins
const int motorStandbyPin = 13;// Pin to enable the motor driver as a whole. Low = Standby, High = Active

const int leftMotorPin1 = 34;
const int leftMotorPin2 = 35;
const int leftMotorPWMPin = 32;

const int rightMotorPin1 = 33;
const int rightMotorPin2 = 25;
const int rightMotorPWMPin = 26;


// Sensor Pins  
const uint8_t sensorPins[] = {23, 22, 1, 3, 21, 19, 18, 5, 17, 16, 4, 0, 2, 15, 13};
const uint8_t sensorCount = 15; // This is NOT a pin.

const uint8_t sensorEmitterPin = 27;// We may want to split this to two pins at some point, depending on performance


// EDF pin
const int edfPin = 12;




//======================================================================================================
//	Structs
//======================================================================================================

#define RING_BUFFER_SIZE 128
// Generic Ring Buffer Struct
struct ringBuffer{
	float buffer[RING_BUFFER_SIZE];
	size_t startIndex;
};





//======================================================================================================
//	Global Variables & Defines
//======================================================================================================

#define SERIAL_ENABLED 1

// WiFi Credentials
#define WIFI_SSID "COLLIN-LAPTOP"
#define WIFI_PASSWORD "blinkyblinky"

// Web Server Variables
AsyncWebServer server(80);// Create AsyncWebServer object on port 80
AsyncWebSocket ws("/ws");// Create a WebSocket object on path "ip:80/ws"


// InfluxDB Credential & info

#define TIME_ZONE "EST5EDT,M3.2.0,M11.1.0"


// Battery Voltage Variables
const uint16_t mainBatHighResistor = 10000;// 10k Ohm Resistor
const uint16_t mainBatLowResistor = 1000;// 1k Ohm Resistor
const uint8_t mainBatMaxVoltage = 8.4;// max voltage for a 2S LiPo Battery
const uint8_t mainBatMinVoltage = 7.3;// min voltage for a 2S LiPo Battery

const uint16_t fanBatHighResistor = 10000;// 10k Ohm Resistor
const uint16_t fanBatLowResistor = 1000;// 1k Ohm Resistor
const uint8_t fanBatMaxVoltage = 8.4;// max voltage for a 2S LiPo Battery
const uint8_t fanBatMinVoltage = 7.3;// min voltage for a 2S LiPo Battery


// Line Sensor Variables
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t sensorLinePosition;


// Motor PWM Properties
const int motorPWMFreq = 5000;// Max frequency of Motor Driver is 100 kHz, so using 5 kHz for now. This will also work with up to a max resolution of 13 bits
const int motorPWMResolution = 8;// 8 bit resolution for PWM. We may want to try a higher resolution at some point
const uint16_t motorMaxSpeed = (1 << motorPWMResolution) - 1;// Max Speed for the motors, 255 for 8 bit PWM
const int leftMotorPWMChannel = 0;
const int rightMotorPWMChannel = 1;


// PID Variables
const uint16_t setpoint = 7000;// Output Value if the line is under the middle sensor

const float Kp = 10.;										// Proportional constant
const float Ki = 0.;										// Integral constant
const float Kd = 4.0;										// Derivative constant

long unclampedOutput;										// Output prior to clamping between 0 and 255 inclusively
bool clamp = 0;												// = 0 if we are not clamping and = 1 if we are clamping
bool saturated = 0;											// = 1 if output is saturated (<=0 or >=255)
bool iClamp;												// Prevents integral windup.   If 0 then continue to add to integral
bool signsEqual;											// = 1 if error and output2 have the same sign
double iError = 0.;											// Integral of error
double dError = 0.;											// Derivative of error


float output2 = 0.;											// Temporary value
int32_t output = 0;											// Output to the motors
float error;												// Setpoint minus measured value


// struct ringBuffer errorBuffer = {{0.}, 127};				// Ring Buffer to store error values
// struct ringBuffer *errorBuffer;							// Ring Buffer to store error values
struct ringBuffer errorBuffer = {0};						// Ring Buffer to store error values


// Web UI Data
JsonDocument messageJSON;// JSON Object to store data to send to the web UI
bool sendingJSON = false;// Variable to keep track of if we are currently sending data to the web UI


// Main Control Loop Wait times
const unsigned long readSensorsTime = 10;		// Read Sensors from thermistor every this many milliseconds
const unsigned long runControllerTime = 10;	// Run the control portion of the code every this many milliseconds
const float runControllerTimeSecs = runControllerTime /1000.;
const unsigned long updateOutputTime = 10;		// Update output every this many milliseconds
const unsigned long logDataTime = 100;			// Send data to the web UI every this many milliseconds

// Main Control Loop run time logs
unsigned long currentMillis = 0;
unsigned long lastReadSensorsTime = 0;
unsigned long lastRunControllerTime = 0;
unsigned long lastUpdateOutputTime = 0;
unsigned long lastLogDataTime = 0;


// Task Delays
const TickType_t batCheckDelay = 30000 / portTICK_PERIOD_MS; // Value in ms
// const TickType_t controlLoopInterval = 10 / portTICK_PERIOD_MS; // Value in ms
const TickType_t wsClientCleanupInterval = 15000 / portTICK_PERIOD_MS; // Value in ms
const TickType_t uiDataSendInterval = 30000 / portTICK_PERIOD_MS; // Value in ms


// Handles for Tasks to run on various cores with different priorities
TaskHandle_t mainControlLoopTask;
TaskHandle_t sendDataToUITask;
TaskHandle_t readBatteryVoltagesTask;


// Event Group for Tasks and related Defines
typedef enum{
	SEND_DATA = 0x01,
	CALIBRATE_SENSOR = 0x02,
	SEND_STATE_INFO = 0x04,
	START_BOT = 0x08,
	STOP_BOT = 0x10,
	START_EDF = 0x20,
	STOP_EDF = 0x40
} EventFlags;
EventGroupHandle_t mainEventGroup;




//======================================================================================================
//	Setup Functions
//======================================================================================================

// Function to setup the WiFi connection
void initWiFi(){
	WiFi.mode(WIFI_STA);// Set WiFi mode to Station (Connecting to some other access point, ie a laptop's hotspot)
	// WiFi.config(IPAddress(192,168,137,2), IPAddress(192,168,137,1),IPAddress(255,255,255,0));// Set Static IP Address (IP, Gateway, Subnet Mask). NOTE: This line does not work on Laptop Hotspot
	WiFi.begin(WIFI_SSID, WIFI_PASSWORD);// Start Wifi Connection
	while(WiFi.status() != WL_CONNECTED){// Wait for WiFi to connect
		delay(500);
		#if SERIAL_ENABLED
			Serial.print(".");
		#endif
	}
	#if SERIAL_ENABLED
		Serial.println("\nConnected to WiFi\n");
	#endif
}



// Function to Calibrate the line following sensor
void calibrateSensor(){
	// This code comes directly from the QTR Sensors library example
	// Ideally, I would like to add a way to display if the sensor is calibrating on the web interface

	// Inform the Web UI that calibration is starting
	while(sendingJSON);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["calibrating"] = true;
	xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI

	// 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
	// = ~25 ms per calibrate() call.
	// Call calibrate() 400 times to make calibration take about 10 seconds.
	for (uint16_t i = 0; i < 400; i++)
	{
		qtr.calibrate();
	}

	// Inform Web UI that calibration is complete
	while(sendingJSON);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["calibrating"] = false;
	xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
}



// Function to setup the motor pins
void setupMotors(){
	// Set Motor Pins to Output
	pinMode(motorStandbyPin, OUTPUT);
	pinMode(leftMotorPin1, OUTPUT);
	pinMode(leftMotorPin2, OUTPUT);
	pinMode(leftMotorPWMPin, OUTPUT);
	pinMode(rightMotorPin1, OUTPUT);
	pinMode(rightMotorPin2, OUTPUT);
	pinMode(rightMotorPWMPin, OUTPUT);

	// Set Motor Standby Pin to Low to Disable the motor driver until we are ready to use it
	digitalWrite(motorStandbyPin, LOW);

	// Set Motor Pins to Values to go forward (bot will not move because standby pin is set low)
	digitalWrite(leftMotorPin1, HIGH);
	digitalWrite(leftMotorPin2, LOW);
	digitalWrite(rightMotorPin1, LOW);
	digitalWrite(rightMotorPin2, HIGH);

	// Set PWM Frequency and Resolution. ledc is the PWM library for the ESP32, since the ESP32 also has a true analog output
	ledcSetup(leftMotorPWMChannel, motorPWMFreq, motorPWMResolution);
	ledcSetup(rightMotorPWMChannel, motorPWMFreq, motorPWMResolution);

	// Attach PWM Channels to Motor Pins
	ledcAttachPin(leftMotorPWMPin, leftMotorPWMChannel);
	ledcAttachPin(rightMotorPWMPin, rightMotorPWMChannel);

	// Set PWM Values to Max Value to start
	ledcWrite(leftMotorPWMChannel, (1 << motorPWMResolution) - 1);
	ledcWrite(rightMotorPWMChannel, (1 << motorPWMResolution) - 1);
}


// Function to setup the EDF




//======================================================================================================
//	Web Socket Server Functions
//======================================================================================================

// Function to send data to all clients on the web UI
void notifyClients(String data) {
	#if SERIAL_ENABLED
		Serial.println("Sending Data to Web UI: " + data + "\n");
	#endif
	ws.textAll(data);
}



// Function to handle the verified content of the WebSocket Messages. This is run in the Web Server thread with priority 3
void selectCommand(char* msg){
		// Check what the message is and set the appropriate bit in the event group
	if (strcmp(msg, "startBot") == 0) {
			xEventGroupSetBits(mainEventGroup, START_BOT);// Set the START_BOT bit
	}else if (strcmp(msg, "stopBot") == 0) {
			xEventGroupSetBits(mainEventGroup, STOP_BOT);// Set the STOP_BOT bit
	}else if (strcmp(msg, "calibrateSensor") == 0) {
			xEventGroupSetBits(mainEventGroup, CALIBRATE_SENSOR);// Set the CALIBRATE_SENSOR bit
	}else if (strcmp(msg, "sendStateInfo") == 0) {
			xEventGroupSetBits(mainEventGroup, SEND_STATE_INFO);// Set the READ_SENSORS bit
	}else if (strcmp(msg, "startEDF") == 0) {
			xEventGroupSetBits(mainEventGroup, START_EDF);// Set the START_EDF bit
	}else if (strcmp(msg, "stopEDF") == 0) {
			xEventGroupSetBits(mainEventGroup, STOP_EDF);// Set the STOP_EDF bit
	}
}



// AwsEventHandler Function
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
	if(type == WS_EVT_CONNECT){
		//client connected
		#if SERIAL_ENABLED
			Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
		#endif
		client->printf("{\"message\": \"Hello Client %u :)\"}", client->id());
		client->ping();
	} else if(type == WS_EVT_DISCONNECT){
		//client disconnected
		#if SERIAL_ENABLED
			Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
		#endif
	} else if(type == WS_EVT_ERROR){
		//error was received from the other end
		#if SERIAL_ENABLED
			Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
		#endif
	} else if(type == WS_EVT_PONG){
		//pong message was received (in response to a ping request maybe)
		#if SERIAL_ENABLED
			Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
		#endif
	} else if(type == WS_EVT_DATA){
		//data packet
		AwsFrameInfo * info = (AwsFrameInfo*)arg;
		if(info->final && info->index == 0 && info->len == len){
			//the whole message is in a single frame and we got all of it's data
			#if SERIAL_ENABLED
				Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
			#endif
			if(info->opcode == WS_TEXT){
				data[len] = 0;// Set last character to null terminator
				#if SERIAL_ENABLED
					Serial.printf("%s\n", (char*)data);
				#endif
				selectCommand((char*)data);
			} else {
				for(size_t i=0; i < info->len; i++){
					#if SERIAL_ENABLED
						Serial.printf("%02x ", data[i]);
					#endif
				}
				#if SERIAL_ENABLED
					Serial.printf("\n");
				#endif
			}
			if(info->opcode == WS_TEXT)
				client->text("{\"message\": \"I got your text message\"}");
			else
				client->binary("{\"message\": \"I got your binary message\"}");
		} else {
			//message is comprised of multiple frames or the frame is split into multiple packets
			if(info->index == 0){
				if(info->num == 0){
					#if SERIAL_ENABLED
						Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
					#endif
				}
				#if SERIAL_ENABLED
					Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
				#endif
			}

			#if SERIAL_ENABLED
				Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);
			#endif
			if(info->message_opcode == WS_TEXT){
				data[len] = 0;
				#if SERIAL_ENABLED
					Serial.printf("%s\n", (char*)data);
				#endif
			} else {
				for(size_t i=0; i < len; i++){
					#if SERIAL_ENABLED
						Serial.printf("%02x ", data[i]);
					#endif
				}
				#if SERIAL_ENABLED
					Serial.printf("\n");
				#endif
			}

			if((info->index + len) == info->len){
				#if SERIAL_ENABLED
					Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
				#endif
				if(info->final){
					#if SERIAL_ENABLED
						Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
					#endif
					if(info->message_opcode == WS_TEXT)
						client->text("{\"message\": \"I got your text message\"}");
					else
						client->binary("{\"message\": \"I got your binary message\"}");
				}
			}
		}
	}
}



void initWebSocket() {
	ws.onEvent(onEvent);
	server.addHandler(&ws);
}




//======================================================================================================
//	Helper Functions
//======================================================================================================

// Function to read a point in the ring buffer
float readPointInBuffer(ringBuffer *buffer, size_t index){
	return buffer->buffer[(buffer->startIndex + index) & 0x7F];
}



// Function to add a point to the end of the ring buffer
void addPointToBuffer(ringBuffer *buffer, float value){
	buffer->buffer[buffer->startIndex] = value;
	buffer->startIndex = (buffer->startIndex - 1) & 0x7F;
	return;
}



//======================================================================================================
//	Control Functions
//======================================================================================================

// PID Controller Function
void calculatePID(){
	error = setpoint - sensorLinePosition; 

	signsEqual = !(((*(uint32_t *) &error) ^ (*(uint32_t *) &output2)) & 0x80000000);	// 1 if error and output2 have the same sign
	iClamp = signsEqual && saturated;													// 1 if the output is saturated AND error and output2 have the same sign
	iError = iError + (error*runControllerTimeSecs) * !iClamp;	// Integral of error
	
	// Calculate Derivative of error
	dError = (error - readPointInBuffer(&errorBuffer, 1))/(2.*runControllerTimeSecs); // Derivative of error using 2nd last point
	dError += (error - readPointInBuffer(&errorBuffer, 3))/(4.*runControllerTimeSecs); // Derivative of error using 4th last point
	dError += (error - readPointInBuffer(&errorBuffer, 7))/(8.*runControllerTimeSecs); // Derivative of error using 8th last point
	dError += (error - readPointInBuffer(&errorBuffer, 15))/(16.*runControllerTimeSecs); // Derivative of error using 16th last point
	dError += (error - readPointInBuffer(&errorBuffer, 31))/(32.*runControllerTimeSecs); // Derivative of error using 32nd last point
	dError += (error - readPointInBuffer(&errorBuffer, 63))/(64.*runControllerTimeSecs); // Derivative of error using 64th last point
	dError += (error - readPointInBuffer(&errorBuffer, 127))/(128.*runControllerTimeSecs); // Derivative of error using 128th last point
	dError = dError/7.; // Average the derivative of error, with all points weighted equally
	addPointToBuffer(&errorBuffer, error);

	// output2 = Kp*error + Ki*iError + Kd*dError + 0.5; // Output value
	output2 = Kp*error*error + Ki*iError + Kd*dError + 0.5; // Output value
	output = output2;

	// Check if Output is Saturated
	if(output >= motorMaxSpeed) // If output is saturated Positive (left)
	{
		output = motorMaxSpeed;
		saturated = 1;
	}
	else if(output <= -motorMaxSpeed) // If output is saturated down (right)
	{
		output = -motorMaxSpeed;
		saturated = 1;
	}
	else
	{
		saturated = 0;
	}
	clamp = iClamp;
	return;
}



// Function to set motor speeds
void setMotorSpeeds(){
	// Set Motor Speeds
	if(output > 0){// If output is positive (the line is to the left of the middle of the sensor), so we need to turn left
		ledcWrite(leftMotorPWMChannel, motorMaxSpeed - output);
		ledcWrite(rightMotorPWMChannel, motorMaxSpeed);
	}else if(output < 0){// If output is negative (the line is to the right of the middle of the sensor), so we need to turn right
		ledcWrite(leftMotorPWMChannel, motorMaxSpeed);
		ledcWrite(rightMotorPWMChannel, motorMaxSpeed + output);
	}else{
		ledcWrite(leftMotorPWMChannel, motorMaxSpeed);
		ledcWrite(rightMotorPWMChannel, motorMaxSpeed);
	}
}



// Function to start the EDF


// Function to stop the EDF


// Function to set the EDF Speed






//======================================================================================================
//	Task Functions
//======================================================================================================

// Task to read battery voltages
void readBatteryVoltages(void *pvParameters){
	while(true){
		// Read Voltage of the Main Battery
		float readVoltage = (analogRead(mainBatPin) / 4096.) * 3.3;// calculate the voltage from the ADC reading
		float mainBatVoltage = readVoltage * (mainBatHighResistor / mainBatLowResistor);// calculate the actual voltage from the voltage divider
		
		// Read Voltage of the Fan Battery
		readVoltage = (analogRead(fanBatPin) / 4096.) * 3.3;// calculate the voltage from the ADC reading
		float fanBatVoltage = readVoltage * (fanBatHighResistor / fanBatLowResistor);// calculate the actual voltage from the voltage divider

		while(sendingJSON);// Wait for the JSON object to be free

		// Add Battery Voltages to JSON object
		messageJSON["mainBatVoltage"] = mainBatVoltage;
		messageJSON["fanBatVoltage"] = fanBatVoltage;
		

		// Delay for 30 seconds
		vTaskDelay(batCheckDelay);
	}
}



// Task to Send Data to Web UI
void sendDataToUI(void *pvParameters){
	#if SERIAL_ENABLED
		Serial.println("Send Data to UI Task Running");
	#endif
	while(true){
		// Wait for the SEND_DATA bit to be set
		EventBits_t bits = xEventGroupWaitBits(mainEventGroup, SEND_DATA, pdTRUE, pdFALSE, uiDataSendInterval);
		if((bits & SEND_DATA) == SEND_DATA){
			#if SERIAL_ENABLED
				Serial.println("sendDataToUI Task: Sending Data to Web UI Triggered\n");
			#endif
			sendingJSON = true;// Set sendingJSON to true to prevent other tasks from modifying the JSON object

			String tmp;
			serializeJson(messageJSON, tmp);
			messageJSON.clear();// Clears the data that was stored in messageJSON, so it will be empty for the next time we want to send data
			sendingJSON = false;// Set sendingJSON to false to allow other tasks to modify the JSON object
			notifyClients(tmp);// Send the JSON data to the web UI
		}
	}
}



// Main Controller Loop, running at Priority 0 on Core 0. It must run at priority 0 to control the core full time
void mainControlLoop(void *pvParameters){
	#if SERIAL_ENABLED
		Serial.println("Main Control Loop Task Running");
	#endif
	while(true){
		// Wait for the START_BOT or CALIBRATE_SENSOR bit to be set
		EventBits_t bits = xEventGroupWaitBits(mainEventGroup, START_BOT | CALIBRATE_SENSOR, pdTRUE, pdFALSE, portMAX_DELAY);

		if((bits & CALIBRATE_SENSOR) != 0){// if the Calibrate Sensor Command was Recieved
			#if SERIAL_ENABLED
				Serial.println("Calibrating Sensor\n");
			#endif
			calibrateSensor();
		}

		if((bits & START_BOT) != 0){// if the Start Bot Command was Recieved
			#if SERIAL_ENABLED
				Serial.println("Starting Bot\n");
			#endif
			// Inform the Web UI that the bot is starting
			while(sendingJSON);// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["botRunning"] = true;
			xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI

			digitalWrite(motorStandbyPin, HIGH);// Set Motor Standby Pin to High to Enable the motor driver

			EventBits_t tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_BOT, pdTRUE, pdFALSE, 0);// Check if the STOP_BOT bit is set

			// Start the Main Control Loop to move the bot, if the STOP_BOT bit is not set
			while((tmpBits & STOP_BOT) == 0){
				// #if SERIAL_ENABLED
				// 	Serial.println("Main Control Loop Running\n");// NOTE: This runs every loop
				// #endif
				currentMillis = millis();
				if(currentMillis - lastReadSensorsTime >= readSensorsTime){
					lastReadSensorsTime = currentMillis;
					// Read Sensor Values
					sensorLinePosition = qtr.readLineBlack(sensorValues);
				}

				currentMillis = millis();
				if(currentMillis - lastRunControllerTime >= runControllerTime){
					lastRunControllerTime = currentMillis;
					// Run PID Controller
					calculatePID();
				}

				currentMillis = millis();
				if(currentMillis - lastUpdateOutputTime >= updateOutputTime){
					lastUpdateOutputTime = currentMillis;
					// Set Motor Speeds
					setMotorSpeeds();
				}

				// Send Data to UI
				currentMillis = millis();
				if(currentMillis - lastLogDataTime >= logDataTime){
					lastLogDataTime = currentMillis;
					// while(sendingJSON);// Wait for the JSON object to be free (not being sent to the web UI)
					// messageJSON["sensorLinePosition"] = sensorLinePosition;
					// xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
				}

				tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_BOT, pdTRUE, pdFALSE, 0);// Check if the STOP_BOT bit is set, no wait time
			}// End of the loop for while the bot is running. Exit if the STOP_BOT bit is set

			// Set Motor Standby Pin to Low to Disable the motor driver
			digitalWrite(motorStandbyPin, LOW);

			// Inform the Web UI that the bot has stopped
			while(sendingJSON);// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["botRunning"] = false;
			xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
		}// End of the Start Bot Command
	}// End of the main control loop infinite loop
}





//======================================================================================================
//	Arduino Main Code
//======================================================================================================

// Setup code, runs once
void setup(){// this will automaticlally run on core 1
	// if SERIAL_ENABLED is defined, then Serial will be enabled
	#if SERIAL_ENABLED
		Serial.begin(115200);
	#endif
	initWiFi();

	// Setup line following sensor
	qtr.setTypeRC();
	qtr.setSensorPins(sensorPins, sensorCount);
	qtr.setEmitterPin(sensorEmitterPin);
	
	// Set up the motor pins
	setupMotors();

	// Set Up WebSocket
	initWebSocket();
	server.begin();// Start the web server. Automatically makes a new task at priority 3, on whatever core is available.

	// Set Up Event Group
	mainEventGroup = xEventGroupCreate();

	// Create Tasks
	// Create Task for reading battery voltages
	xTaskCreatePinnedToCore(
		readBatteryVoltages,	 // Task Function
		"Read Battery Voltages", // Task Name
		10000,					// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
		NULL,					// Parameters
		1,						// Priority 1
		&readBatteryVoltagesTask,// Task Handle
		1						// Core 1
	);

	// Create Task for sending data to the web UI
	xTaskCreatePinnedToCore(
		sendDataToUI,		// Task Function
		"Send Data to Server",	// Task Name
		10000,					// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
		NULL,					// Parameters
		2,						// Priority 2
		&sendDataToUITask,		// Task Handle
		1						// Core 1
	);

	// Create Task for the main control loop
	xTaskCreatePinnedToCore(
		mainControlLoop,	 // Task Function
		"Main Control Loop", // Task Name
		10000,				// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
		NULL,				// Parameters
		0,					// Priority 0
		&mainControlLoopTask,// Task Handle
		0					// Core 0
	);




	// Functions that may be needed in the future
	// xTaskCreatePinnedToCore;
	// xTaskCreateUniversal;
	// xEventGroupWaitBits;
	// EventBits_t;
	// xSemaphoreCreateMutex;
	// WS_EVT_CONNECT;
	// map();

	// xTaskDelayUntil();
	// vTaskDelayUntil();// Older version of the above function
	// vTaskDelay();
	// taskYIELD();
	// vTaskGetRunTimeStats;

	// ws.cleanupClients();// This should be run occasionally somewhere
}



// Main Code Loop
void loop(){// this will automatically run on core 1
	ws.cleanupClients();// This will be run occasionally to clean up the web socket clients
	#if SERIAL_ENABLED
		// Serial.println("Send Data to UI Task High Water Mark:" + uxTaskGetStackHighWaterMark(sendDataToUITask));
	#endif
	vTaskDelay(wsClientCleanupInterval);// Delay for 15 seconds
}




