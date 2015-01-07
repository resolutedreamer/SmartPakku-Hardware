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

// store state of 4 FSRs
uint8_t state;

// store forces
long fsrForces[4];

// what data to send
int bt_transfer_seq;

long measure_force(int fsrPin)
{
	int fsrReading;     // the analog reading from the FSR resistor divider
	int fsrVoltage;     // the analog reading converted to voltage
	unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
	unsigned long fsrConductance; 
	long fsrForce;       // Finally, the resistance converted to force

	fsrReading = analogRead(fsrPin);

	fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
	if (fsrVoltage == 0)
	{
		fsrForce = 0;
	}
	else
	{
		fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
		fsrResistance *= 10000;                // 10K resistor
		fsrResistance /= fsrVoltage;

		fsrConductance = 1000000;           // we measure in micromhos so 
		fsrConductance /= fsrResistance;

		// Use the two FSR guide graphs to approximate the force
		if (fsrConductance <= 1000)
		{
			fsrForce = fsrConductance / 80;
		}
		else
		{
			fsrForce = fsrConductance - 1000;
			fsrForce /= 30;
		}
	}
	return fsrForce;
}

void process_forces()
{
	fsrForces[0] = measure_force(fsrPin0);
	fsrForces[1] = measure_force(fsrPin1);
	fsrForces[2] = measure_force(fsrPin2);
	fsrForces[3] = measure_force(fsrPin3);
}

bool fsr_pressed(long fsrForce)
{
	if (fsrForce > 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t get_pressed_state()
{
	state = 0;
	bool fsrPressed[4];
	
	process_forces();
	
	fsrPressed[0] = fsr_pressed(fsrForces[0]);
	fsrPressed[1] = fsr_pressed(fsrForces[1]);
	fsrPressed[2] = fsr_pressed(fsrForces[2]);
	fsrPressed[3] = fsr_pressed(fsrForces[3]);
	
	if (fsrPressed[0])
	{
		state = state | 1;
		
	}
	if (fsrPressed[1])
	{
		state = state | 2;
		
	}
	if (fsrPressed[2])
	{
		state = state | 4;
		
	}
	if (fsrPressed[3])
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
	bt_transfer_seq = 0;

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

uint8_t assign_value()
{
	switch(bt_transfer_seq)
	{
		case 1:
		   return 's';
		case 2:
		   return get_pressed_state();
		case 3:
		   return fsrForces[0];
		case 4:
		   return fsrForces[1];
		case 5:
		   return fsrForces[2];
		case 6:
		   return fsrForces[3];
		default:
		   return 0;
	}
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
		
		// The data that should be sent will depend on what stage of sending we are at
		temp[1] = assign_value();
		if (bt_transfer_seq <= 6)
		{
			bt_transfer_seq += 1;
		}
		else
		{
			bt_transfer_seq = 0;
		}
		//
		Serial.print("We are in bt_transfer_seq stage: ");
		Serial.print(bt_transfer_seq);
		//
		Serial.print("The value we are sending is: ");
		Serial.print(temp[1]);
		
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










/*
void get_FSR_raw_values(int fsrRawData[])
{
	int fsrRawData0 = analogRead(fsrPin0);  
	int fsrRawData1 = analogRead(fsrPin1);
	int fsrRawData2 = analogRead(fsrPin2);
	int fsrRawData3 = analogRead(fsrPin3);
	
	fsrRawData[0] = fsrRawData0;
	fsrRawData[1] = fsrRawData1;
	fsrRawData[2] = fsrRawData2;
	fsrRawData[3] = fsrRawData3;
}

void get_FSR_voltages(int fsrVoltage[])
{
	int fsrVoltage0 = map(fsrRawData0, 0, 1023, 0, 5000);
	int fsrVoltage1 = map(fsrRawData1, 0, 1023, 0, 5000);
	int fsrVoltage2 = map(fsrRawData2, 0, 1023, 0, 5000);
	int fsrVoltage3 = map(fsrRawData3, 0, 1023, 0, 5000);
	
	fsrVoltage[0] = fsrVoltage0;
	fsrVoltage[1] = fsrVoltage1;
	fsrVoltage[2] = fsrVoltage2;
	fsrVoltage[3] = fsrVoltage3;
}

long get_FSR_force(int fsrVoltage)
{
	unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
	unsigned long fsrConductance; 
	long fsrForce;       // Finally, the resistance converted to force	
	
	if (fsrVoltage == 0)
	{
		fsrForce = 0;
	}
	
	else
	{
		// The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
		// so FSR = ((Vcc - V) * R) / V        yay math!
		fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
		fsrResistance *= 10000;                // 10K resistor
		fsrResistance /= fsrVoltage;
		
		fsrConductance = 1000000;           // we measure in micromhos so 
		fsrConductance /= fsrResistance;
		
		// Use the two FSR guide graphs to approximate the force
		if (fsrConductance <= 1000)
		{
			fsrForce = fsrConductance / 80;
		}
		else
		{
			fsrForce = fsrConductance - 1000;
			fsrForce /= 30;
		}
	}
	return fsrForce;
}

void process_forces(int fsrVoltage[])
{
	// This is held globally
	// long fsrForces[4];
	
	fsrForces[0] = get_force(fsrVoltage[0]);
	fsrForces[1] = get_force(fsrVoltage[1]);
	fsrForces[2] = get_force(fsrVoltage[2]);
	fsrForces[3] = get_force(fsrVoltage[3]);
	
}

*/




