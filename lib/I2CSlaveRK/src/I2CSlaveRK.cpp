
#include "Particle.h"
#include "I2CSlaveRK.h"

static I2CSlave *globalObj = 0;

I2CSlave::I2CSlave(TwoWire &wire, uint8_t addr, size_t numRegisters) :
		wire(wire), addr(addr), numRegisters(numRegisters),
		nextReadAddr(0), registerSetFlags(0) {
	registers = (uint32_t *)malloc(numRegisters * sizeof(uint32_t));
	globalObj = this;
}

I2CSlave::~I2CSlave() {
	free(registers);
	globalObj = 0;
}

void I2CSlave::begin() {

	wire.begin(addr);
	wire.onReceive(receiveEventStatic);
	wire.onRequest(requestEventStatic);
}

uint32_t I2CSlave::getRegister(uint16_t regAddr) const {
	if (regAddr < numRegisters) {
		return registers[regAddr];
	}
	else {
		return 0;
	}
}

void I2CSlave::setRegister(uint16_t regAddr, uint32_t value) {
	if (regAddr < numRegisters) {
		registers[regAddr] = value;
	}
}

bool I2CSlave::getRegisterSet(uint16_t &regAddr) {
	if (registerSetFlags) {
		for(uint16_t ii = 0; ii < 32; ii++) {
			if (registerSetFlags & (1 << ii)) {
				// Bit is set, return the address and clear the flag
				regAddr = ii;
				registerSetFlags &= ~(1 << ii);
				return true;
			}
		}
		return false;
	}
	else {
		return false;
	}
}


void I2CSlave::receiveEvent(int numBytes) {

	if (numBytes == sizeof(uint16_t)) {
		// This is just an  address to read the register. Save for requestEvent.
		nextReadAddr = (wire.read() & 0xff);
		nextReadAddr |= (wire.read() & 0xff) << 8;
	}
	else
	if (numBytes == (sizeof(uint16_t) + sizeof(uint32_t))) {
		// Set a register
		uint16_t writeAddr = (wire.read() & 0xff);
		writeAddr |= (wire.read() & 0xff) << 8;

		uint32_t value = (wire.read() & 0xff);
		value |= (wire.read() & 0xff) << 8;
		value |= (wire.read() & 0xff) << 16;
		value |= (wire.read() & 0xff) << 24;

		if (writeAddr < numRegisters) {
			registers[writeAddr] = value;

			if (writeAddr < 32) {
				registerSetFlags |= 1 << writeAddr;
			}
		}
	}

}

void I2CSlave::requestEvent() {
	// Request to read
	uint32_t value = 0;

	if (nextReadAddr < numRegisters) {
		value = registers[nextReadAddr];
	}
	wire.write((const uint8_t *)&value, sizeof(value));
}

// [static]
void I2CSlave::receiveEventStatic(int numBytes) {
	if (globalObj) {
		globalObj->receiveEvent(numBytes);
	}
}

// [static]
void I2CSlave::requestEventStatic() {
	if (globalObj) {
		globalObj->requestEvent();
	}
}
