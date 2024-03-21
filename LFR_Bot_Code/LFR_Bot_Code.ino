/*
	This Code is made for the "BotGoBrrr" Group (Group 2) Line Following Robot.
	Code was wrote in majority by Collin Schofield, with assistance from Github Copilot.
	Also, much of the basis for the networking code came from Rui Santos's ESP32 tutorials: https://randomnerdtutorials.com/projects-esp32/
*/

// #define CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS	1


#define SERIAL_ENABLED 0
#define NO_WIRELESS 0
#define ENABLE_EDF 1
#define HALF_SENSORS 0

#include <Arduino.h>

#if (!NO_WIRELESS)
#include <WiFi.h>
#define CONFIG_ASYNC_TCP_RUNNING_CORE 1 // Set the core to run the Async TCP library on
#include <AsyncTCP.h>// Used in the ESPAsyncWebServer library, included here to guarnatee the define is set as we want. 
#include <ESPAsyncWebServer.h>// This will automatically use whatever core is available, at priority 3
	#include <ElegantOTA.h>// Library for Over The Air (OTA) updates
	// #include <AsyncUDP.h>// Library for UDP communication
// #include <HTTPClient.h>
// #include <WebServer.h>
// #include <InfluxDbClient.h>
#endif

#include <ArduinoJson.h>// Library for JSON parsing. This is a more efficient alternative to the arduino built in JSON library
#include <QTRSensors.h>// Library for the line following sensors
#include <Preferences.h>// Library to store data in the ESP32's flash memory. This will be used for persistent storage of the PID constants, when saveConstants is called

#if (ENABLE_EDF == 1)
	#include <DShotRMT.h>// Library for the DShot protocol, used to control the EDF
#elif (ENABLE_EDF == 2)
	#include <ESC.h>// Library for the DShot protocol, used to control the EDF
#endif




//======================================================================================================
//	Hardware Connections
//======================================================================================================
// GPIO 6-11 are for the flash interface. They seem to be connected to flash chip and should probably be avoided

// Battery Analog Reading Pins
const gpio_num_t mainBatPin = GPIO_NUM_36;// GPIO36 - ADC 0
const gpio_num_t fanBatPin = GPIO_NUM_39;// GPIO39 - ADC 3

const gpio_num_t leftMotorPWMPin = GPIO_NUM_22;
const gpio_num_t rightMotorPWMPin = GPIO_NUM_23;

// Sensor Pins  
#if HALF_SENSORS
	// const uint8_t sensorPins[] = {18, 26, 5, 25, 17, 32, 16, 27, 4, 14, 21, 12, 2, 13, 15};
	const uint8_t sensorPins[] =    {18,    5,    17,     16,     4,     21,     2,     15};
	const uint8_t sensorCount = 8; // This is NOT a pin.
#else
	const uint8_t sensorPins[] = {18, 26, 5, 25, 17, 32, 16, 27, 4, 14, 21, 12, 2, 13, 15};
const uint8_t sensorCount = 15; // This is NOT a pin.
#endif

const gpio_num_t sensorEmitterPinOdd = GPIO_NUM_19;
const gpio_num_t sensorEmitterPinEven = GPIO_NUM_33;


// EDF pin
const gpio_num_t edfPin = GPIO_NUM_0;




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

// WiFi Credentials
// #define WIFI_SSID "COLLIN-LAPTOP"
// #define WIFI_PASSWORD "blinkyblinky"
#define WIFI_SSID "Techman"
#define WIFI_PASSWORD "Camo2070"

// Web Server Variables
AsyncWebServer server(80);// Create AsyncWebServer object on port 80
AsyncWebSocket ws("/ws");// Create a WebSocket object on path "ip:80/ws"


// InfluxDB Credential & info

#define TIME_ZONE "EST5EDT,M3.2.0,M11.1.0"


// Preferences Object to store PID Constants
Preferences preferences;


// Battery Voltage Variables
const float mainBatHighResistor = 49790.;// 50K Ohm Resistor
const float mainBatLowResistor = 10035.;// 10K Ohm Resistor
const float mainBatMaxVoltage = 12.6;// max voltage for a 3S LiPo Battery
const float mainBatMinVoltage = 10.2;// min voltage for a 3S LiPo Battery

const float fanBatHighResistor = 29815.;// 30k Ohm Resistor
const float fanBatLowResistor = 9961.6;// 10k Ohm Resistor
const float fanBatMaxVoltage = 8.4;// max voltage for a 2S LiPo Battery
const float fanBatMinVoltage = 6.8;// min voltage for a 2S LiPo Battery


// Line Sensor Variables
QTRSensors qtr;
uint16_t sensorValues[sensorCount];
uint16_t sensorLinePosition;
uint16_t checkedSensorLinePosition;
#if HALF_SENSORS
	const uint16_t sensorCheckThreshold = 0;11112``7q`3
	const QTRReadMode sensorReadMode = QTRReadMode::On;// Read mode for the QTR Sensors. This is the default mode, and it reads the odd and even sensors, and turns off the rest
#else
const uint16_t sensorCheckThreshold = 6000;// Threshold for the sensor to be considered on the line. This is the default value, and may need to be adjusted
	const QTRReadMode sensorReadMode = QTRReadMode::On;// Read mode for the QTR Sensors. This is the default mode, and it reads the odd and even sensors, and turns off the rest
#endif


// Motor PWM Properties
const uint16_t motorPWMFreq = 5000;// Max frequency of Motor Driver is 100 kHz, so using 5 kHz for now. This will also work with up to a max resolution of 13 bits
const uint8_t motorPWMResolution = 10;// 10 bit resolution for PWM. We may want to try a higher resolution at some point
const uint16_t motorAbsMaxSpeed = (1 << (motorPWMResolution)) - 1;// Absolute maximum speed of the motor. 1023 for 10 bit PWM, 4095 for 12 bit PWM. This is the maximum value that can be sent to the motor driver
uint16_t motorMaxSpeed = 700;// Limited maximum PWM speed for the motors. This can be changed from the web UI.
uint16_t newMotorMaxSpeed = motorMaxSpeed;// New maximum speed for the motors, when we want to change it
bool updateMotorMaxSpeed = false;// = 1 if we want to update the maximum speed of the motors
const uint8_t leftMotorPWMChannel = 0;
const uint8_t rightMotorPWMChannel = 1;


// PID Variables
#if HALF_SENSORS
	const uint16_t setpoint = 3500;// Output Value if the line is under the middle sensor
#else
const uint16_t setpoint = 7000;// Output Value if the line is under the middle sensor
#endif

const float defaultKp = 0.9;							// Default Proportional constant
const float defaultKp2 = 0.0;							// Default Proportional^2 constant
const float defaultKi = 0.0;								// Default Integral constant
const float defaultKd = 0.43;								// Default Derivative constant

float Kp = defaultKp;										// Proportional constant
float Kp2 = defaultKp2;										// Proportional^2 constant
float Ki = defaultKi;										// Integral constant
float Kd = defaultKd;										// Derivative constant

float newKp = Kp;									// New Proportional constant, when we want to change it
float newKp2 = Kp2;									// New Proportional^2 constant, when we want to change it
float newKi = Ki;									// New Integral constant, when we want to change it
float newKd = Kd;									// New Derivative constant, when we want to change it

bool updateConstants = false;								// = 1 if we want to update the PID constants

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

float errorSign;											// Sign of the error


struct ringBuffer errorBuffer = {0};						// Ring Buffer to store error values


// Web UI Data
JsonDocument messageJSON;			// JSON Object to store data to send to the web UI
SemaphoreHandle_t jsonSemaphore;	// Semaphore to control access to the JSON object


// Main Control Loop Wait times
const unsigned long readSensorsTime = 7;		// Read Sensors from thermistor every this many milliseconds
const unsigned long runControllerTime = readSensorsTime;		// Run the control portion of the code every this many milliseconds
const float runControllerTimeSecs = runControllerTime /1000.;
const unsigned long updateOutputTime = readSensorsTime;		// Update output every this many milliseconds
const unsigned long logDataTime = 1000;			// Send data to the web UI every this many milliseconds

// Main Control Loop run time logs
unsigned long currentMillis = 0;
unsigned long lastReadSensorsTime = 0;
unsigned long lastRunControllerTime = 0;
unsigned long lastUpdateOutputTime = 0;
unsigned long lastLogDataTime = 0;

// Main Control loop chunk run time logs
unsigned long readSensorsTimeTaken = 0;
unsigned long runControllerTimeTaken = 0;
unsigned long updateOutputTimeTaken = 0;

bool logRunTimes = false;// = 1 if we want to log the run times of the main control loop


// EDF Variables
#if (ENABLE_EDF >= 1)
	typedef enum{
		EDF_UNINITIALIZED = 0,
		EDF_STOPPED = 1,
		EDF_RUNNING = 2
	} EDFState;

	EDFState edfState = EDF_UNINITIALIZED;

	#if NO_WIRELESS
		const TickType_t edfRunTime = pdMS_TO_TICKS(45000);								// Time for the EDF to Run before shutting off, currently 45 seconds
	#endif
#endif
#if (ENABLE_EDF == 1)
	const rmt_channel_t edfRmtChannel = RMT_CHANNEL_3;
	DShotRMT edf;

	const uint16_t edfMinSpeed = 48;							// Minimum Speed of the ducted fan. This is the speed sent for if the fan is off.
	const uint16_t edfAbsMaxSpeed = 2047;						// Absolute Maximum Speed of the Ducted Fan
	uint16_t edfMaxSpeed = 800;									// Current Limited Target Speed of the EDF
	uint16_t newEDFMaxSpeed = edfMaxSpeed;						// New Speed of the EDF, updated from the Web UI

	uint16_t edfCurrentSpeed = edfMinSpeed;						// Current actual set speed of the EDF
	TickType_t lastEDFCommandTime = 0;							// Time of the last command sent to the EDF
#endif
#if (ENABLE_EDF == 2)
	const uint16_t edfMinSpeed = 1000;							// Minimum Speed of the ducted fan. This is the speed sent for if the fan is off.
	const uint16_t edfAbsMaxSpeed = 2000;						// Absolute Maximum Speed of the Ducted Fan
	uint16_t edfMaxSpeed = 1100;									// Current Limited Target Speed of the EDF
	uint16_t newEDFMaxSpeed = edfMaxSpeed;						// New Speed of the EDF, updated from the Web UI

	uint16_t edfCurrentSpeed = edfMinSpeed;						// Current actual set speed of the EDF


	ESC edf (edfPin, edfMinSpeed, edfAbsMaxSpeed, 500); // Create an ESC object: ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
#endif


// Task Delays
const TickType_t batCheckDelay = 15000 / portTICK_PERIOD_MS; // Value in ms
// const TickType_t controlLoopInterval = 10 / portTICK_PERIOD_MS; // Value in ms
const TickType_t wsClientCleanupInterval = 15000 / portTICK_PERIOD_MS; // Value in ms
const TickType_t uiDataSendInterval = 45000 / portTICK_PERIOD_MS; // Value in ms


// Handles for Tasks to run on various cores with different priorities
TaskHandle_t mainControlLoopTask;
TaskHandle_t sendDataToUITask;
TaskHandle_t readBatteryVoltagesTask;
#if (ENABLE_EDF >= 1)
	TaskHandle_t edfControlTask;
#endif
#if (ENABLE_EDF == 1)
	TaskHandle_t edfSpeedControlTask;
#endif

// Event Group for Tasks and related Defines
typedef enum{
	SEND_DATA = 0x01,
	CALIBRATE_SENSOR = 0x02,
	INIT_EDF = 0x04,
	START_BOT = 0x08,
	STOP_BOT = 0x10,
	START_EDF = 0x20,
	STOP_EDF = 0x40,
	CHANGE_EDF_SPEED = 0x80,
	BEGIN_EDF_SPEED_TASK = 0x100,
	STOP_EDF_SPEED_TASK = 0x200
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
	// This code comes directly from the QTR Sensors library example, with the addition of the logging, sensorReadMode, and resetting the calibration.

	// Inform the Web UI that calibration is starting
	#if (!NO_WIRELESS)
		xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["calibrating"] = true;
		xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
	xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	#endif

	qtr.resetCalibration(); // Reset the calibration, so we can start fresh each time the button is pressed.

	// 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
	// = ~25 ms per calibrate() call.
	// Call calibrate() 400 times to make calibration take about 10 seconds.
	for (uint16_t i = 0; i < 400; i++)
	{
		qtr.calibrate(sensorReadMode);
	}

	#if (!NO_WIRELESS)
	// Inform Web UI that calibration is complete
		xSemaphoreTake(jsonSemaphore,portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["calibrating"] = false;
		xSemaphoreGive(jsonSemaphore);// Give the JSON object back
	xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	#endif
}



// Function to setup the motor pins
void setupMotors(){
	// Set Motor Pins to Output
	pinMode(leftMotorPWMPin, OUTPUT);
	pinMode(rightMotorPWMPin, OUTPUT);

	// Set PWM Frequency and Resolution. ledc is the PWM library for the ESP32, since the ESP32 also has a true analog output
	ledcSetup(leftMotorPWMChannel, motorPWMFreq, motorPWMResolution);
	ledcSetup(rightMotorPWMChannel, motorPWMFreq, motorPWMResolution);

	// Attach PWM Channels to Motor Pins
	ledcAttachPin(leftMotorPWMPin, leftMotorPWMChannel);
	ledcAttachPin(rightMotorPWMPin, rightMotorPWMChannel);

	// Set PWM Values to Max Value to start
	ledcWrite(leftMotorPWMChannel, 0);
	ledcWrite(rightMotorPWMChannel, 0);
}



// Function to setup the EDF
#if (ENABLE_EDF >= 1)
void initEDF(){
	#if (ENABLE_EDF == 1)
		edf.install(edfPin, edfRmtChannel);	// Install the DShotRMT object to the pin and RMT channel
		edf.init();							// Initialize the DShotRMT EDF

		xEventGroupSetBits(mainEventGroup, BEGIN_EDF_SPEED_TASK);// Set the BEGIN_EDF_SPEED_TASK bit to start the EDF speed control task, to maintain the EDF speed (Starting with the EDF off)
	#endif
	#if (ENABLE_EDF == 2)
		edf.arm();							// Arm the EDF
		edf.speed(edfMinSpeed);			// Set the EDF to the minimum speed
		edfCurrentSpeed = edfMinSpeed;		// Set the current speed of the EDF to the minimum speed
	#endif

	edfState = EDF_STOPPED;				// Set the EDF state to stopped
	#if (!NO_WIRELESS)
		xSemaphoreTake(jsonSemaphore, portMAX_DELAY);								// Wait for the JSON object to be free (not being sent to the web UI)
		messageJSON["edfState"] = edfState;
		xSemaphoreGive(jsonSemaphore);											// Give the JSON object back to the main control loop
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	#endif
}
#endif





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
	JsonDocument doc; // Create a JSON document to store the message
	DeserializationError error = deserializeJson(doc, msg);// Deserialize the JSON message

	if(error){// If there was an error deserializing the JSON message
		#if SERIAL_ENABLED
			Serial.print(F("deserializeJson() failed: "));
			Serial.println(error.c_str());
		#endif
		ws.textAll("{\"message\": \"Invalid JSON received\"}");// Send a message to the web UI that the JSON was invalid
		return;
	}

	const char* cmd = doc["cmd"];// Get the command from the JSON message

		// Check what the message is and set the appropriate bit in the event group
	if (strcmp(cmd, "startBot") == 0) {
		xEventGroupSetBits(mainEventGroup, START_BOT);			// Set the START_BOT bit
	}else if (strcmp(cmd, "stopBot") == 0) {
		xEventGroupSetBits(mainEventGroup, STOP_BOT);			// Set the STOP_BOT bit
	}else if (strcmp(cmd, "calibrateSensor") == 0) {
		xEventGroupSetBits(mainEventGroup, CALIBRATE_SENSOR);	// Set the CALIBRATE_SENSOR bit
#if (ENABLE_EDF >= 1)
	}else if (strcmp(cmd, "initEDF") == 0) {
		xEventGroupSetBits(mainEventGroup, INIT_EDF);			// Set the READ_SENSORS bit
	}else if (strcmp(cmd, "startEDF") == 0) {
		xEventGroupSetBits(mainEventGroup, START_EDF);			// Set the START_EDF bit
	}else if (strcmp(cmd, "stopEDF") == 0) {
		xEventGroupSetBits(mainEventGroup, STOP_EDF);			// Set the STOP_EDF bit
	}else if (strcmp(cmd, "setEDFSpeed") == 0) {
		newEDFMaxSpeed = doc["newEDFSpeed"];
		xEventGroupSetBits(mainEventGroup, CHANGE_EDF_SPEED);	// Set the CHANGE_EDF_SPEED bit
	}else if (strcmp(cmd, "getEDFInfo") == 0) {					// Send the current state of the EDF to the web UI
		getEDFInfo();
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
#endif
	}else if (strcmp(cmd, "updatePID") == 0) {
		newKp = doc["Kp"];
		newKp2 = doc["Kp2"];
		newKi = doc["Ki"];
		newKd = doc["Kd"];
		updateConstants = true;// Tell the PID control function to update the constants on its next run
	}else if (strcmp(cmd, "getPID") == 0) {
		// Send the current PID constants to the web UI
		getPIDConstants();
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}else if (strcmp(cmd, "savePID") == 0) {
		// Save the PID constants to the flash memory
		preferences.begin("PID", false);// Open the preferences object with the namespace "PID" and read/write access
		preferences.putFloat("Kp", Kp);// Save the Kp constant to the flash memory
		preferences.putFloat("Kp2", Kp2);// Save the Kp2 constant to the flash memory
		preferences.putFloat("Ki", Ki);// Save the Ki constant to the flash memory
		preferences.putFloat("Kd", Kd);// Save the Kd constant to the flash memory
		preferences.end();// Close the preferences object
	}else if (strcmp(cmd, "readSensor") == 0) {
		// Read the sensor values and send them to the web UI
		xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
		messageJSON["sensorLinePosition"] = qtr.readLineBlack(sensorValues, sensorReadMode);// Read the sensor values and add them to the JSON object
		xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}else if (strcmp(cmd, "cleariError") == 0) {
		iError = 0.;// Clear the integral error
	}else if (strcmp(cmd, "runTimeLogOn") == 0) {
		logRunTimes = true;								// Enable logging of the run times of the main control loop
		xSemaphoreTake(jsonSemaphore, portMAX_DELAY);	// Wait for the JSON object to be free (not being sent to the web UI
		messageJSON["logRunTimes"] = logRunTimes;		// Add the logRunTimes value to the JSON object
		xSemaphoreGive(jsonSemaphore);					// Give the JSON object back to the main control loop
		xEventGroupSetBits(mainEventGroup, SEND_DATA);	// Set the SEND_DATA bit to send data to the web UI
	}else if (strcmp(cmd, "runTimeLogOff") == 0) {
		logRunTimes = false;							// Disable logging of the run times of the main control loop
		xSemaphoreTake(jsonSemaphore, portMAX_DELAY);	// Wait for the JSON object to be free (not being sent to the web UI
		messageJSON["logRunTimes"] = logRunTimes;		// Add the logRunTimes value to the JSON object
		xSemaphoreGive(jsonSemaphore);					// Give the JSON object back to the main control loop
		xEventGroupSetBits(mainEventGroup, SEND_DATA);	// Set the SEND_DATA bit to send data to the web UI
	}else if (strcmp(cmd, "updateMotorMaxSpeed") == 0) {
		newMotorMaxSpeed = doc["newMotorMaxSpeed"];
		updateMotorMaxSpeed = true;// Tell the motor control function to update the maximum speed on its next run
	}else if (strcmp(cmd, "getMotorMaxSpeed") == 0) {
		// Send the current maximum motor speed to the web UI
		getMotorMaxSpeed();
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}else if (strcmp(cmd, "getState") == 0) {
		// Send the current state of the bot to the web UI
		readBatteryVoltages();
		getPIDConstants();
		getMotorMaxSpeed();
		#if (ENABLE_EDF >= 1)
			getEDFInfo();
		#endif
		messageJSON["logRunTimes"] = logRunTimes;
		// messageJSON["message"] = "getState command received. This is a test message";
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}else {
		// If the command is not recognized, send a message to the web UI
		ws.textAll("{\"message\": \"Invalid Command Received\"}");
		// xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}
}



// AwsEventHandler Function. This largely comes from the ESPAsyncWebServer Documentation example.
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
			// if(info->opcode == WS_TEXT)
			// 	client->text("{\"message\": \"I got your text message\"}");
			// else
			// 	client->binary("{\"message\": \"I got your binary message\"}");
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
					// if(info->message_opcode == WS_TEXT)
					// 	client->text("{\"message\": \"I got your text message\"}");
					// else
					// 	client->binary("{\"message\": \"I got your binary message\"}");
				}
			}
		}
	}
}



void initWebSocket() {
	ws.onEvent(onEvent);
	server.addHandler(&ws);
}



// New Printf function for the program to use for logging via the web UI console, rather than serial
int asyncLogPrintf(const char *format, va_list args) {
	char buffer[256]; // Buffer to hold the formatted message
	vsnprintf(buffer, sizeof(buffer), format, args); // Format the message
	String message = "{\"message\": \"" + String(buffer) + "\"}"; // Add the JSON structure
	message.replace("\n", "\\n"); // Replace newline characters with the escape sequence, as newlines are not valid in the JSON string

	ws.textAll(message.c_str()); // Send the message to all WebSocket clients
	return strlen(buffer); // Return the length of the formatted message
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



// Get the current PID constants for Web UI
void getPIDConstants(){
	xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)

	JsonObject consts = messageJSON["consts"].to<JsonObject>();
	consts["kp"] = Kp;
	consts["kp2"] = Kp2;
	consts["ki"] = Ki;
	consts["kd"] = Kd;
	
	xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
}



// Function to read the battery voltages
void readBatteryVoltages(){
	// Read Voltage of the Main Battery
	float readVoltage = (analogRead(mainBatPin) / 4095.) * 3.3;// calculate the voltage from the ADC reading
	float mainBatVoltage = readVoltage * ((mainBatHighResistor + mainBatLowResistor) / mainBatLowResistor);// calculate the actual voltage from the voltage divider
	
	// Read Voltage of the Fan Battery
	readVoltage = (analogRead(fanBatPin) / 4095.) * 3.3;// calculate the voltage from the ADC reading
	float fanBatVoltage = readVoltage * ((fanBatHighResistor + fanBatLowResistor) / fanBatLowResistor);// calculate the actual voltage from the voltage divider

	xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free
	// Add Battery Voltages to JSON object
	messageJSON["mainBatVoltage"] = mainBatVoltage;
	messageJSON["fanBatVoltage"] = fanBatVoltage;
	xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
}



// Get the motor maximum speed for the Web UI
void getMotorMaxSpeed(){
	xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["motorMaxSpeed"] = motorMaxSpeed;
	messageJSON["motorAbsMaxSpeed"] = motorAbsMaxSpeed;
	xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
}



// Get the EDF maximum speed and current state for the Web UI
#if (ENABLE_EDF >= 1)
void getEDFInfo(){
	xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
	messageJSON["edfMinSpeed"] = edfMinSpeed;
	messageJSON["edfMaxSpeed"] = edfMaxSpeed;
	messageJSON["edfAbsMaxSpeed"] = edfAbsMaxSpeed;
	messageJSON["edfState"] = edfState;
	xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
}
#endif





//======================================================================================================
//	Control Functions
//======================================================================================================

// PID Controller Function
void calculatePID(){
	if(updateConstants){// If new constant values were received
		Kp = newKp;
		Kp2 = newKp2;
		Ki = newKi;
		Kd = newKd;
		updateConstants = false;
	}

	error = setpoint - checkedSensorLinePosition; 

	if(error < 0){
		errorSign = -1;
	}
	else{
		errorSign = 1;
	}

	signsEqual = !(((*(uint32_t *) &error) ^ (*(uint32_t *) &output2)) & 0x80000000);	// 1 if error and output2 have the same sign
	iClamp = signsEqual && saturated;													// 1 if the output is saturated AND error and output2 have the same sign
	iError = iError + (error*runControllerTimeSecs) * !iClamp;	// Integral of error
	
	// Calculate Derivative of error
	// dError = (error - lastError)/(1.*runControllerTime/1000.); // Derivative of error
	// dError = (error - lastError2)/(2.*runControllerTime/1000.); // Derivative of error
	// dError = 3.*((error - readPointInBuffer(&errorBuffer, 1))/(2.*runControllerTimeSecs)); // Derivative of error using 2nd last point
	// dError = ((error - readPointInBuffer(&errorBuffer, 1))/(2.*runControllerTimeSecs)); // Derivative of error using 2nd last point
	dError = 0.01 * ((error - readPointInBuffer(&errorBuffer, 0))/(1.*runControllerTimeSecs)); // Derivative of error using last point
	dError += 1.99 * ((error - readPointInBuffer(&errorBuffer, 1))/(2.*runControllerTimeSecs)); // Derivative of error using 2nd last point
	// dError += (error - readPointInBuffer(&errorBuffer, 3))/(4.*runControllerTimeSecs); // Derivative of error using 4th last point
	// dError += (error - readPointInBuffer(&errorBuffer, 7))/(8.*runControllerTimeSecs); // Derivative of error using 8th last point
	// dError += (error - re`adPointInBuffer(&errorBuffer, 15))/(16.*runControllerTimeSecs); // Derivative of error using 16th last point
	// dError += (error - readPointInBuffer(&errorBuffer, 31))/(32.*runControllerTimeSecs); // Derivative of error using 32nd last point
	// dError += (error - readPointInBuffer(&errorBuffer, 63))/(64.*runControllerTimeSecs); // Derivative of error using 64th last point
	// dError += (error - readPointInBuffer(&errorBuffer, 127))/(128.*runControllerTimeSecs); // Derivative of error using 128th last point
	dError = dError/2.; // Average the derivative of error, with all points weighted equally
	addPointToBuffer(&errorBuffer, error);

	output2 = Kp*error + Kp2*error*error*errorSign + Ki*iError + Kd*dError + 0.5; // Output value
	output = output2;	// Saving the Float output value to an Integer, to be used in the motor control function

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
	if(updateMotorMaxSpeed){// If new maximum speed values were received
		motorMaxSpeed = newMotorMaxSpeed;
		updateMotorMaxSpeed = false;
	}
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



#if (ENABLE_EDF >= 1)
// Function to ramp the EDF speed up or down. This function came from the DShotRMT library example, with some modifications
void rampEDFThrottle(int start, int stop, int step)
{
	if (step == 0)
		return;

	for (int i = start; step > 0 ? i < stop : i > stop; i += step)
	{
		#if (ENABLE_EDF == 1)
			// edf.sendThrottle(i);
			edfCurrentSpeed = i;
			vTaskDelay(1);
		#endif
		#if (ENABLE_EDF == 2)
			edf.speed(i);
			edfCurrentSpeed = i;
			vTaskDelay(5);
		#endif
	}
	#if (ENABLE_EDF == 1)
		// edf.sendThrottle(stop);
		edfCurrentSpeed = stop;
	#endif
	#if (ENABLE_EDF == 2)
		edf.speed(stop);
		edfCurrentSpeed = stop;
	#endif
}



// Function to start the EDF
void startEDF(){
	if(edfState == EDF_STOPPED){
		edfState = EDF_RUNNING;
		rampEDFThrottle(edfMinSpeed, edfMaxSpeed, 1);
		#if (!NO_WIRELESS)
			xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["edfState"] = edfState;
			xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
			xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
		#endif
	}
}



// Function to stop the EDF
void stopEDF(){
	if(edfState == EDF_RUNNING){
		rampEDFThrottle(edfMaxSpeed, edfMinSpeed, -5);
		edfState = EDF_STOPPED;
		#if (!NO_WIRELESS)
			xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["edfState"] = edfState;
			xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
			xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
		#endif
	}
}
#endif// ENABLE_EDF





//======================================================================================================
//	Task Functions
//======================================================================================================

// Task to read battery voltages
void readBatteryVoltagesLoop(void *pvParameters){
	while(true){
		readBatteryVoltages();// Read the battery voltages

		// Set the SEND_DATA bit to send data to the web UI
		xEventGroupSetBits(mainEventGroup, SEND_DATA);

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

			xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI

			String tmp;
			serializeJson(messageJSON, tmp);
			messageJSON.clear();// Clears the data that was stored in messageJSON, so it will be empty for the next time we want to send data
			xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop

			notifyClients(tmp);// Send the JSON data to the web UI
		}
	}
}



// Main Controller Loop, running at Priority 0 on Core 0. It must run at priority 0 to control the core full time
void mainControlLoop(void *pvParameters){
	#if SERIAL_ENABLED
		Serial.println("Main Control Loop Task Running");
	#endif
	#if (!NO_WIRELESS)
	while(true){
		// Wait for the START_BOT or CALIBRATE_SENSOR bit to be set
		EventBits_t bits = xEventGroupWaitBits(mainEventGroup, START_BOT | CALIBRATE_SENSOR, pdTRUE, pdFALSE, portMAX_DELAY);

		if((bits & CALIBRATE_SENSOR) != 0){// if the Calibrate Sensor Command was Received
			#if SERIAL_ENABLED
				Serial.println("Calibrating Sensor\n");
			#endif
			calibrateSensor();
		}

		if((bits & START_BOT) != 0){// if the Start Bot Command was Received
			#if SERIAL_ENABLED
				Serial.println("Starting Bot\n");
			#endif
			// Inform the Web UI that the bot is starting
			xSemaphoreTake(jsonSemaphore, portMAX_DELAY);// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["botRunning"] = true;
			xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
			xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI

			ledcWrite(leftMotorPWMChannel, motorMaxSpeed);// Set Motor Speeds to the maximum speed
			ledcWrite(rightMotorPWMChannel, motorMaxSpeed);// Set Motor Speeds to the maximum speed

			EventBits_t tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_BOT, pdTRUE, pdFALSE, 0);// Check if the STOP_BOT bit is set

			// Start the Main Control Loop to move the bot, if the STOP_BOT bit is not set
			while((tmpBits & STOP_BOT) == 0){
	#endif
				// #if SERIAL_ENABLED
				// 	Serial.println("Main Control Loop Running\n");// NOTE: This runs every loop
				// #endif
				currentMillis = millis();
				if(currentMillis - lastReadSensorsTime >= readSensorsTime){
					lastReadSensorsTime = currentMillis;
					// Read Sensor Values
					sensorLinePosition = qtr.readLineBlack(sensorValues, sensorReadMode);
				#if HALF_SENSORS
					if((sensorLinePosition == 0 && checkedSensorLinePosition >= (7000 - sensorCheckThreshold)) || (sensorLinePosition == 7000 && checkedSensorLinePosition <= sensorCheckThreshold)){
				#else
					if((sensorLinePosition == 0 && checkedSensorLinePosition >= (14000 - sensorCheckThreshold)) || (sensorLinePosition == 14000 && checkedSensorLinePosition <= sensorCheckThreshold)){
				#endif
						// If the sensor values went from one extreme to the other, then the line is probably lost, so we should hold the last known position
						// checkedSensorLinePosition = checkedSensorLinePosition;
					}else{
						checkedSensorLinePosition = sensorLinePosition;
					}
					readSensorsTimeTaken = millis() - lastReadSensorsTime;// Log the time taken to read the sensors
				}

				currentMillis = millis();
				if(currentMillis - lastRunControllerTime >= runControllerTime){
					lastRunControllerTime = currentMillis;
					// Run PID Controller
					calculatePID();
					runControllerTimeTaken = millis() - lastRunControllerTime;// Log the time taken to run the controller
				}

				currentMillis = millis();
				if(currentMillis - lastUpdateOutputTime >= updateOutputTime){
					lastUpdateOutputTime = currentMillis;
					// Set Motor Speeds
					setMotorSpeeds();
					updateOutputTimeTaken = millis() - lastUpdateOutputTime;// Log the time taken to update the motor speeds
				}

				// Send Data to UI
				currentMillis = millis();
				if(currentMillis - lastLogDataTime >= logDataTime){
					lastLogDataTime = currentMillis;
					BaseType_t status = xSemaphoreTake(jsonSemaphore, pdMS_TO_TICKS(readSensorsTime/4));// Wait for the JSON object to be free (not being sent to the web UI)
					if(status == pdTRUE){// If the JSON object was successfully taken
						// Add the sensor line position to the JSON object
					if(logRunTimes){// If we want to log the run times of the main control loop
						// Add the run times to the JSON object (in milliseconds)
						JsonObject runTimes = messageJSON["runTimes"].to<JsonObject>();
						runTimes["sensorReadTime"] = readSensorsTimeTaken;
						runTimes["calcPIDTime"] = runControllerTimeTaken;
						runTimes["updateOutputTime"] = updateOutputTimeTaken;
						}
						xSemaphoreGive(jsonSemaphore);// Give the JSON object back to the main control loop
						xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
					}
				}

	#if (!NO_WIRELESS)
				tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_BOT, pdTRUE, pdFALSE, 0);// Check if the STOP_BOT bit is set, no wait time
			}// End of the loop for while the bot is running. Exit if the STOP_BOT bit is set

			// Stop the bot
			ledcWrite(leftMotorPWMChannel, 0);
			ledcWrite(rightMotorPWMChannel, 0);

			// Inform the Web UI that the bot has stopped
			xSemaphoreTake(jsonSemaphore, portMAX_DELAY);	// Wait for the JSON object to be free (not being sent to the web UI)
			messageJSON["botRunning"] = false;
			xSemaphoreGive(jsonSemaphore);					// Give the JSON object back
			xEventGroupSetBits(mainEventGroup, SEND_DATA);	// Set the SEND_DATA bit to send data to the web UI
		}// End of the Start Bot Command
	}// End of the main control loop infinite loop
	#endif
}



#if (ENABLE_EDF >= 1)
// EDF Control Loop Task, Running at Priority 1 on Core 1
void edfMainControlLoop(void *pvParameters){
	#if SERIAL_ENABLED
		Serial.println("EDF Control Loop Task Running");
	#endif
	while(true){
		// Wait for the INIT_EDF, START_EDF or STOP_EDF bit to be set
		EventBits_t bits = xEventGroupWaitBits(mainEventGroup, INIT_EDF | START_EDF | STOP_EDF | CHANGE_EDF_SPEED, pdTRUE, pdFALSE, portMAX_DELAY);

		if((bits & INIT_EDF) != 0){// if the Init EDF Command was Received
			#if SERIAL_ENABLED
				Serial.println("Initializing EDF\n");
			#endif
			// Initialize the EDF
			initEDF();
		}

		if((bits & START_EDF) != 0){// if the Start EDF Command was Received
			#if SERIAL_ENABLED
				Serial.println("Starting EDF\n");
			#endif
			// Start the EDF
			startEDF();
			#if NO_WIRELESS
				vTaskDelay(edfRunTime);
				xEventGroupSetBits(mainEventGroup, STOP_EDF);
			#endif
		}

		if((bits & STOP_EDF) != 0){// if the Stop EDF Command was Received
			#if SERIAL_ENABLED
				Serial.println("Stopping EDF\n");
			#endif
			// Stop the EDF
			stopEDF();
		}

		if((bits & CHANGE_EDF_SPEED) != 0){// if the Change EDF Speed Command was Received
			#if SERIAL_ENABLED
				Serial.println("Changing EDF Speed\n");
			#endif
			// Change the EDF Max Speed
			if(edfState == EDF_RUNNING){// If the EDF is running
				rampEDFThrottle(edfCurrentSpeed, newEDFMaxSpeed, 1);
			}
			edfMaxSpeed = newEDFMaxSpeed;
		}
	}
}
#endif // ENABLE_EDF >= 1



#if (ENABLE_EDF == 1)
// EDF Speed Setting Loop Task, Running at Priority 2 on Core 1
void edfSpeedCmdLoop(void *pvParameters){
	#if SERIAL_ENABLED
		Serial.println("EDF Speed Setting Loop Task Running");
	#endif
	while(true){
		// Wait for the SET_EDF_SPEED bit to be set
		EventBits_t bits = xEventGroupWaitBits(mainEventGroup, BEGIN_EDF_SPEED_TASK, pdTRUE, pdFALSE, portMAX_DELAY);

		if((bits & BEGIN_EDF_SPEED_TASK) != 0){								// if the Command to start this loop was Received
			lastEDFCommandTime = xTaskGetTickCount();															// Set the last EDF command time to the current time
			EventBits_t tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_EDF_SPEED_TASK, pdTRUE, pdFALSE, 0);	// Check if the STOP_EDF bit is set

			while((tmpBits & STOP_EDF_SPEED_TASK) == 0){														// Start the loop for while the EDF is running. Exit if the STOP_EDF_SPEED_TASK bit is set
				edf.sendThrottle(edfCurrentSpeed);																// Set the EDF Speed
				tmpBits = xEventGroupWaitBits(mainEventGroup, STOP_EDF_SPEED_TASK, pdTRUE, pdFALSE, 0);			// Check if the STOP_EDF_SPEED_TASK bit is set
				xTaskDelayUntil(&lastEDFCommandTime, pdMS_TO_TICKS(1));											// Delay for 1 millisecond from the start of this loop, to have a 1kHz update rate
			}
		}
	}
}
#endif // ENABLE_EDF == 1





//======================================================================================================
//	Arduino Main Code
//======================================================================================================

// Setup code, runs once
void setup(){// this will automaticlally run on core 1
	esp_log_set_vprintf(asyncLogPrintf);// Change the location the default logging goes to to the asyncLogPrintf function, rather than printing to serial. 
	
	// if SERIAL_ENABLED is defined, then Serial will be enabled
	#if SERIAL_ENABLED
		Serial.begin(115200);
	#endif
	#if (!NO_WIRELESS)
		initWiFi();// Initialize the WiFi connection, if the NO_WIRELESS flag is not set
		jsonSemaphore = xSemaphoreCreateMutex();// Create a semaphore to control
		if( jsonSemaphore != NULL ) {
			// The semaphore was created successfully.
			// The semaphore can now be used.
		}
	#endif

	// Setup line following sensor
	qtr.setTypeRC();
	qtr.setSensorPins(sensorPins, sensorCount);
	qtr.setEmitterPins(sensorEmitterPinOdd, sensorEmitterPinEven);

	// Set up the PID constants
	preferences.begin("PID", false);// Open the preferences object for the PID constants
	Kp = preferences.getFloat("Kp", defaultKp);
	Kp2 = preferences.getFloat("Kp2", defaultKp2);
	Ki = preferences.getFloat("Ki", defaultKi);
	Kd = preferences.getFloat("Kd", defaultKd);
	preferences.end();// Close the preferences object

	#if (!NO_WIRELESS)
		if(Kp != defaultKp || Kp2 != defaultKp2 || Ki != defaultKi || Kd != defaultKd){// If the PID constants were found in the flash memory
		// Send the PID constants to the web UI
		JsonObject consts = messageJSON["consts"].to<JsonObject>();
		consts["kp"] = Kp;
			consts["kp2"] = Kp2;
		consts["ki"] = Ki;
		consts["kd"] = Kd;
		messageJSON["message"] = "PID Constants Loaded from Flash Memory";
		xEventGroupSetBits(mainEventGroup, SEND_DATA);// Set the SEND_DATA bit to send data to the web UI
	}
	#endif
	
	// Set up the motor pins
	setupMotors();

	// Set Up WebSocket
	#if (!NO_WIRELESS)
		ElegantOTA.begin(&server);
	initWebSocket();
	server.begin();// Start the web server. Automatically makes a new task at priority 3, on whatever core is available.
	#endif

	// Set Up Event Group
	mainEventGroup = xEventGroupCreate();

	#if (!NO_WIRELESS)
	// Create Tasks
	// Create Task for reading battery voltages
	xTaskCreatePinnedToCore(
		readBatteryVoltagesLoop,	 // Task Function
		"Read Battery Voltages", // Task Name
		1024,					// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
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
			3,						// Priority 3, so it is at the same priority as the recieving data task. This is to help prevent the two from getting locked up
		&sendDataToUITask,		// Task Handle
		1						// Core 1
	);

	// Create Task for the main control loop
	xTaskCreatePinnedToCore(
		mainControlLoop,	 // Task Function
		"Main Control Loop", // Task Name
		10000,				// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
		NULL,				// Parameters
		0,					// Priority 0, so it can run continuously
		&mainControlLoopTask,// Task Handle
		0					// Core 0. This task should get this core to itself
	);
	#endif // !NO_WIRELESS

	#if (ENABLE_EDF >= 1)
		// Create the main EDF control loop task
		xTaskCreatePinnedToCore(
			edfMainControlLoop,	 // Task Function
			"EDF Control Loop", // Task Name
			10000,				// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
			NULL,				// Parameters
			1,					// Priority 1
			&edfControlTask,// Task Handle
			#if NO_WIRELESS
				0				// Core 0 if there is no wireless
			#else
				1				// Core 1 if there is wireless, so the main control loop can run on core 0
	#endif
		);
	#endif // ENABLE_EDF >= 1

	#if (ENABLE_EDF == 1)
		// Create the EDF Speed Setting Loop Task
		xTaskCreatePinnedToCore(
			edfSpeedCmdLoop,	 // Task Function
			"EDF Speed Setting Loop", // Task Name
			5000,				// Stack Size, should check utilization later with uxTaskGetStackHighWaterMark
			NULL,				// Parameters
			2,					// Priority 2
			&edfSpeedControlTask,// Task Handle
			#if NO_WIRELESS
				0				// Core 0 if there is no wireless
			#else
				1				// Core 1 if there is wireless, so the main control loop can run on core 0
			#endif
		);
	#endif // ENABLE_EDF == 1

	#if NO_WIRELESS
		calibrateSensor();
		#if (ENABLE_EDF >= 1)
			xEventGroupSetBits(mainEventGroup, INIT_EDF);
			xEventGroupSetBits(mainEventGroup, START_EDF);// Start the EDF. When NO_WIRELESS is defined, this will make the EDF run for the duration of edfRunTime then automatically stop
		#endif
		vTaskDelay(pdMS_TO_TICKS(1000));// Delay for 1 second
		ledcWrite(leftMotorPWMChannel, motorMaxSpeed);// Set Motor Speeds to the maximum speed
		ledcWrite(rightMotorPWMChannel, motorMaxSpeed);// Set Motor Speeds to the maximum speed
	#endif




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



// "Main" Code Loop. This is used to monitor the stack high water marks and free heap, and to clean up the web socket clients
void loop(){// this will automatically run on core 1
	#if (NO_WIRELESS)
		mainControlLoop(NULL);
	#else
		ws.cleanupClients();// This will be run occasionally to clean up the web socket clients
		// #if SERIAL_ENABLED
		// 	Serial.printf("Send Data to UI Task High Water Mark: %u\n", uxTaskGetStackHighWaterMark(sendDataToUITask));
		// 	Serial.printf("Main Control Loop Task High Water Mark: %u\n", uxTaskGetStackHighWaterMark(mainControlLoopTask));
		// 	Serial.printf("Read Battery Voltages Task High Water Mark: %u\n", uxTaskGetStackHighWaterMark(readBatteryVoltagesTask));
		// 	Serial.printf("Free Heap: %u\n", ESP.getFreeHeap());
		// #else
		// 	ws.printfAll("{\"message\": \"Send Data to UI Task High Water Mark: %u\\nMain Control Loop Task High Water Mark: %u\\nRead Battery Voltages Task High Water Mark: %u\\nFree Heap: %u\\n\"}", uxTaskGetStackHighWaterMark(sendDataToUITask), uxTaskGetStackHighWaterMark(mainControlLoopTask), uxTaskGetStackHighWaterMark(readBatteryVoltagesTask), ESP.getFreeHeap());
		// #endif
	vTaskDelay(wsClientCleanupInterval);// Delay for 15 seconds
	#endif
}




