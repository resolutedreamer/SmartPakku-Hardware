// Set Serial Monitor to 115200
// Currently this will output large amounts of debugging data

#define NRF_DEBUG 1
#include <SPI.h>
#include <nRF8001.h>

#include "services.h"

// For LiPo Fuel Gauge
#include "MAX17043.h"
#include "Wire.h"

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

// change nRF8001 reset pin to -1 if it's not connected
// Redbear BLE Shield users: to my knowledge reset pin is not connected so use -1!
// NOTE: if you choose -1, youll need to manually reset your device after powerup!!
#define RESET_PIN 9
#define REQN_PIN 10
#define RDYN_PIN 2

nRF8001 *nrf;

float temperatureC;
uint8_t pipeStatusReceived, dataSent;
unsigned long lastSent;

// For LiPo Fuel Gauge
MAX17043 batteryMonitor;

// For FSR

// each of a0 - a3 have a FSR and 10K pulldown resistor attached 
const int fsrPin0 = 0;
const int fsrPin1 = 1;
const int fsrPin2 = 2;
const int fsrPin3 = 3;     

// store readings for each of the pins
int fsrRawData0;
int fsrRawData1;
int fsrRawData2;
int fsrRawData3;

// store state of 4 4 resistors
uint8_t state;

void get_FSR_values()
{
	fsrRawData0 = analogRead(fsrPin0);  
	fsrRawData1 = analogRead(fsrPin1);
	fsrRawData2 = analogRead(fsrPin2);
	fsrRawData3 = analogRead(fsrPin3);
}

bool fsr_pressed(int fsrRawData)
{
	if (fsrRawData > 800)
	{
		return true;
	}
	else
        {
        	return false;
        }
}

uint8_t get_state()
{
	state = 0;
	bool fsr0pressed, fsr1pressed, fsr2pressed, fsr3pressed;
	get_FSR_values();
	fsr0pressed = fsr_pressed(fsrRawData0);
	fsr1pressed = fsr_pressed(fsrRawData1);
	fsr2pressed = fsr_pressed(fsrRawData2);
	fsr3pressed = fsr_pressed(fsrRawData3);
	if (fsr0pressed)
	{
		state = state | 1;
	}
	if (fsr1pressed)
	{
		state = state | 2;
	}
	if (fsr2pressed)
	{
		state = state | 4;
	}
	if (fsr3pressed)
	{
		state = state | 8;
	}
	return state;
}


// This function is called when nRF8001 responds with the temperature
void temperatureHandler(float tempC)
{
	Serial.println("received temperature");
	temperatureC = tempC;
}

// Generic event handler, here it's just for debugging all received events
void eventHandler(nRFEvent *event)
{
	Serial.println("event handler");
	nrf->debugEvent(event);
}

void setup()
{
	temperatureC = 0.0;
	pipeStatusReceived = 0;
	lastSent = 0;

	Wire.begin();
	Serial.begin(115200);
	Serial.println("Hello");
	
	// LiPo Fuel Gauge Init Code
	
	Serial.println("MAX17043 Example: reading voltage and SoC");
	Serial.println();

	batteryMonitor.reset();
	batteryMonitor.quickStart();
	delay(1000);

	float cellVoltage = batteryMonitor.getVCell();
	Serial.print("Voltage:\t\t");
	Serial.print(cellVoltage, 4);
	Serial.println("V");

	float stateOfCharge = batteryMonitor.getSoC();
	Serial.print("State of charge:\t");
	Serial.print(stateOfCharge);
	Serial.println("%");

	// // LiPo Fuel Gauge Init Code End
	

	// nRF8001 class initialized with pin numbers
	nrf = new nRF8001(RESET_PIN, REQN_PIN, RDYN_PIN);

	// Register event handles
	nrf->setEventHandler(&eventHandler);
	nrf->setTemperatureHandler(&temperatureHandler);
	if ((nrf->setup(setup_msgs, NB_SETUP_MESSAGES)) == cmdSuccess)
	{
		Serial.println("SUCCESS");
	}
	else
	{
		Serial.println("FAIL");
		while (1);
	}

	// These functions merely request device address and temperature,
	// actual responses are asynchronous. They'll return error codes
	// if somehow the request itself failed, for example because
	// the device is not ready for these commands.
	nrf->getDeviceAddress();
	nrf->poll();
	nrf->getTemperature();
	nrf->poll();

	if (temperatureC > 0.0) {
	Serial.print("Temperature: ");
	Serial.println(temperatureC, 2);
	}

	nrf->connect(0, 32);
}

void loop()
{
	Serial.println("polling");

	// Polling will block - times out after 2 seconds
	nrf->poll(2000);

	// If heart rate pipe is open
	if (nrf->isPipeOpen(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX) && (millis() - lastSent) > 1000 && temperatureC > 0.0 && nrf->creditsAvailable())
	{
		Serial.println("ready to send data");
		uint8_t temp[2];
		temp[0] = 0;
		//temp[1] = round(temperatureC);
		
		// For determing backpack state  
		temp[1] = get_state();
		
		nrf->sendData(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX, 2, (uint8_t *)&temp);
		lastSent = millis();
		
		// For LiPo Fuel Gauge
		uint8_t bat = batteryMonitor.getSoC();		

		// If battery pipe is open
		if (nrf->isPipeOpen(PIPE_BATTERY_BATTERY_LEVEL_TX) && nrf->creditsAvailable())
		{
			nrf->sendData(PIPE_BATTERY_BATTERY_LEVEL_TX, 1, &bat);
		}

		// get new temperature
		nrf->getTemperature();
	}
	else if (nrf->getConnectionStatus() == Disconnected)
	{
		Serial.println("Reconnecting");
		dataSent = 0;
		nrf->connect(0, 32);
	}
}
