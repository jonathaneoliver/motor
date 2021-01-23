#include "Particle.h"

#include "I2CSlaveMasterRK.h"

I2CSlaveMaster device(Wire, 0x10);

const unsigned long CHECK_PERIOD_MS = 1000;
unsigned long lastCheck = 0;

uint32_t counter = 0;


void setup() {
	Serial.begin(9600);
	device.begin();
}

void loop() {

	if (millis() - lastCheck >= CHECK_PERIOD_MS) {
		lastCheck = millis();

		uint32_t reg0;
		if (!device.readRegister(0, reg0)) {
			Serial.println("read register 0 failed");
			return;
		}

		Serial.printlnf("slave 0 = %u", reg0);

		Serial.printlnf("setting 1 to %u", counter);
		if (!device.writeRegister(1, counter++)) {
			Serial.println("write register 1 failed");
			return;
		}

		int r = analogRead(A0);
		Serial.printlnf("setting 2 to %u", r);
		if (!device.writeRegister(2, r)) {
			Serial.println("write register 2 failed");
			return;
		}
	}
}
