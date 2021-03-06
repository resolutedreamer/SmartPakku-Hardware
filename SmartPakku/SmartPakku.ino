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

// For FSR
// each of a0 - a3 have a FSR and 10K pulldown resistor attached 
#define fsrPin0 0
#define fsrPin1 1
#define fsrPin2 2
#define fsrPin3 3

nRF8001 *nrf;

float temperatureC;
uint8_t pipeStatusReceived, dataSent;
unsigned long lastSent;

// For LiPo Fuel Gauge
MAX17043 batteryMonitor;

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

void process_forces(long fsrForces[])
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

uint8_t get_pressed_state(long fsrForces[])
{
	uint8_t state = 0;
	bool fsrPressed0, fsrPressed1, fsrPressed2, fsrPressed3;

	// store forces
	process_forces(fsrForces);
	
	fsrPressed0 = fsr_pressed(fsrForces[0]);
	fsrPressed1 = fsr_pressed(fsrForces[1]);
	fsrPressed2 = fsr_pressed(fsrForces[2]);
	fsrPressed3 = fsr_pressed(fsrForces[3]);
	
	if (fsrPressed0)
	{
		state = state | 1;
		
	}
	if (fsrPressed1)
	{
		state = state | 2;
		
	}
	if (fsrPressed2)
	{
		state = state | 4;
		
	}
	if (fsrPressed3)
	{
		state = state | 8;
		
	}
	return state;
}

uint8_t assign_value(int bt_transfer_seq, long fsrForces[])
{
	uint8_t a_state;
	uint8_t a_force;
	String sending = "Sending: ";
	String sending_weight = "We send a weight in Newtons";
			

	Serial.print("We are in bt_transfer_seq stage: ");
	Serial.println(bt_transfer_seq);
	switch(bt_transfer_seq)
	{
		case 0:
			Serial.println("Send an 's'");			
			Serial.print(sending);
			Serial.println("s");		
			return 's';
		case 1:
			a_state = get_pressed_state(fsrForces);
			Serial.println("We send the current backpack state.");			
			Serial.print(sending);
			Serial.println(a_state);		
			return a_state;
		case 2:
			a_force = fsrForces[0];
			Serial.println(sending_weight);			
			Serial.print(sending);
			Serial.println(a_force);		
			return a_force;
		case 3:
			a_force = fsrForces[1];
			Serial.println(sending_weight);			
			Serial.print(sending);
			Serial.println(a_force);		
			return a_force;
		case 4:
			a_force = fsrForces[2];
			Serial.println(sending_weight);			
			Serial.print(sending);
			Serial.println(a_force);		
			return a_force;
		case 5:
			a_force = fsrForces[3];
			Serial.println(sending_weight);			
			Serial.print(sending);
			Serial.println(a_force);		
			return a_force;
		default:
			Serial.println("FAILURE");
			return 0;
	}
}


// This function is called when nRF8001 responds with the temperature
void temperatureHandler(float tempC)
{
	//Serial.println("received temperature");
	temperatureC = tempC;
}

// Generic event handler, here it's just for debugging all received events
void eventHandler(nRFEvent *event)
{
	//Serial.println("event handler");
	nrf->debugEvent(event);
}

void setup()
{
	temperatureC = 0.0;
	pipeStatusReceived = 0;
	lastSent = 0;

	Wire.begin();
	Serial.begin(115200);
	//Serial.println("Hello");
	
	// LiPo Fuel Gauge Init Code
	
	//Serial.println("MAX17043 Example: reading voltage and SoC");
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
	// what data to send
	int bt_transfer_seq = 0;
	long fsrForces[4];
	Serial.println("Looping");

	// Polling will block - times out after 2 seconds
	nrf->poll(2000);

	bool a = nrf->isPipeOpen(PIPE_HEART_RATE_HEART_RATE_MEASUREMENT_TX);
	bool b = (millis() - lastSent) > 1000;
	bool c = temperatureC > 0.0 && nrf->creditsAvailable();
	
	Serial.println(a);
	Serial.println(b);
	Serial.println(c);
	Serial.println();
	
	
	// If heart rate pipe is open
	if ( a && b && c)
	{
		Serial.println("Preparing to send");
		uint8_t temp[2];
		temp[0] = 0;
		//temp[1] = round(temperatureC);
		
		// The data that should be sent will depend on what stage of sending we are at
		temp[1] = assign_value(bt_transfer_seq, fsrForces);
		if (bt_transfer_seq > 6)
		{
			bt_transfer_seq = 0;
		}
		else
		{
			bt_transfer_seq += 1;
		}
		
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




