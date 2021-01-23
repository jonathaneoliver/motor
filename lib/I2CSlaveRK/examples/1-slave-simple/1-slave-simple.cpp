#include "Particle.h"

#include "I2CSlaveRK.h"

// Set up this Photon as an I2C device device, address 0x10, with 10 uint32 registers
I2CSlave device(Wire, 0x10, 10);

unsigned long lastCounterUpdate = 0;
uint32_t counter = 0;

void setup() {
	Serial.begin(9600);

	device.begin();
}

void loop() {

	if (millis() - lastCounterUpdate >= 1000) {
		// Once per second increment register 0
		lastCounterUpdate = millis();

		Serial.printlnf("register 0 set to %u", counter);
		device.setRegister(0, counter++);
	}

	uint16_t regAddr;
	while(device.getRegisterSet(regAddr)) {
		// regAddr was updated from the I2C master
		Serial.printlnf("master updated %u to %u", regAddr, device.getRegister(regAddr));
	}
}
