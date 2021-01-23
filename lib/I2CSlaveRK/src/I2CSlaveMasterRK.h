#ifndef __I2CSLAVEMASTERRK_H
#define __I2CSLAVEMASTERRK_H

#include "Particle.h"

/**
 * Class to communicate with an I2CSlaveRK Photon/Electron device
 *
 * The use of this class is optional, but it encapsulates the peculariaties of commnicating with one.
 */
class I2CSlaveMaster {
public:
	/**
	 * Create an object to communicate with a slave device
	 *
	 * wire is the I2C interface it is on, typically Wire (D0/D1). On the Electron it can be Wire1 (C4/C5).
	 *
	 * addr is the 7-bit I2C address of the slave device you want to communicate with.
	 */
	I2CSlaveMaster(TwoWire &wire, uint8_t addr);
	virtual ~I2CSlaveMaster();

	/**
	 * You must call begin() from setup() to initialize the Wire interface.
	 */
	void begin();

	/**
	 * Write to a register on the slave device.
	 *
	 * regAddr the register address. The slave determines the number of addresses. Note that this is
	 * a register number, not a memory address, so 0 is the first one, 1 is the second 32-bit register, etc.
	 *
	 * value is the 32-bit value to write.
	 *
	 * Returns true if the call succeeded or false if there was an I2C error.
	 */
	bool writeRegister(uint16_t regAddr, uint32_t value);

	/**
	 * Read a register on the slave device
	 *
	 * regAddr the register address. The slave determines the number of addresses. Note that this is
	 * a register number, not a memory address, so 0 is the first one, 1 is the second 32-bit register, etc.
	 *
	 * value is filled in with the value retrieved from the slave device.
	 *
	 * Returns true if the call succeeded or false if there was an I2C error.
	 */
	bool readRegister(uint16_t regAddr, uint32_t &value);

protected:
	TwoWire &wire;
	uint8_t addr;
};


#endif /* __I2CSLAVEMASTERRK_H */

