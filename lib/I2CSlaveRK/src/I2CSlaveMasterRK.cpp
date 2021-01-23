
#include "Particle.h"

#include "I2CSlaveMasterRK.h"

I2CSlaveMaster::I2CSlaveMaster(TwoWire &wire, uint8_t addr) : wire(wire), addr(addr) {

}

I2CSlaveMaster::~I2CSlaveMaster() {
}

void I2CSlaveMaster::begin() {
	wire.begin();
}

bool I2CSlaveMaster::writeRegister(uint16_t regAddr, uint32_t value) {
	wire.beginTransmission(addr);

	wire.write((const uint8_t *)&regAddr, sizeof(regAddr));
	wire.write((const uint8_t *)&value, sizeof(value));

	return wire.endTransmission(true) == 0;
}

bool I2CSlaveMaster::readRegister(uint16_t regAddr, uint32_t &value) {
	wire.beginTransmission(addr);
	wire.write((const uint8_t *)&regAddr, sizeof(regAddr));
	int res = wire.endTransmission(false);
	if (res != 0) {
		// Serial.printlnf("endTransmission error res=%d", res);
		return false;
	}

	wire.requestFrom(addr, (uint8_t) sizeof(value), (uint8_t) true);

	if (wire.available() != sizeof(value)) {
		// Serial.printlnf("wrong data size returned");
		return false;
	}

	// little-endian
	value = (wire.read() & 0xff);
	value |= (wire.read() & 0xff) << 8;
	value |= (wire.read() & 0xff) << 16;
	value |= (wire.read() & 0xff) << 24;

	return true;
}
