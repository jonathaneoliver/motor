#ifndef __I2CSLAVE_RK_H
#define __I2CSLAVE_RK_H

/**
 * Class for making a Photon/Electron be an I2C slave device
 *
 * This is opposite of the usual case; normally the Photon is the master to slave devices
 * like sensors and displays.
 *
 * Note: You can only instantiate one of these because of the way the event handlers
 * are registered.
 */
class I2CSlave {
public:
	/**
	 * Initialize the object. Normally you have one of these as a global variable.
	 *
	 * wire is the Wire interface to use, typically Wire (D0/D1). On the Electron it can be Wire1 (C4/C5)
	 *
	 * addr is the 7-bit I2C address of this slave device.
	 *
	 * numRegisters is the number of 32-bit registers to allocate space for.
	 */
	I2CSlave(TwoWire &wire, uint8_t addr, size_t numRegisters);
	virtual ~I2CSlave();

	/**
	 * The begin method must be called from setup()
	 */
	void begin();

	/**
	 * Gets the value of a register
	 *
	 * regAddr must be 0 <= regAddr < numRegisters
	 *
	 * The value is typically set by the I2C master, though you can also read values you've set
	 * locally on this machine. See also getRegisterSet.
	 */
	uint32_t getRegister(uint16_t regAddr) const;

	/**
	 * Sets the value of a register
	 *
	 * regAddr must be 0 <= regAddr < numRegisters
	 *
	 * value is a 32-bit value, not interpreted in any way
	 *
	 * You normally set a register so it can be retrieved from the I2C master. The model of this class
	 * is that you store the value in a register whenever it is changed, as this is a low-cost operation.
	 *
	 * This class doesn't support a model where the value is instantaenously generated when the I2C
	 * master requests it as you don't have much time to to this and it requires annoying ISR programming.
	 */
	void setRegister(uint16_t regAddr, uint32_t value);

	/**
	 * Whenever the I2C master sets a register, a flag is set (for the first 32 registers).
	 *
	 * If you are interested in knowing when this happens, call getRegisterSet() from your loop. If it
	 * returns true, then regAddr will be filled in with the register that was set. You can then use
	 * getRegister() to get the value, if you care about that.
	 *
	 * You can also use this merely as a signal. For example, if the master writes to register 4, you
	 * might interpret this as a trigger to do something.
	 */
	bool getRegisterSet(uint16_t &regAddr);

protected:
	void receiveEvent(int numBytes);
	static void receiveEventStatic(int numBytes);

	void requestEvent();
	static void requestEventStatic();

protected:
	TwoWire &wire;
	uint8_t addr; // 7-bit address for this device
	size_t numRegisters;
	uint32_t *registers;
	uint16_t nextReadAddr;
	volatile uint32_t registerSetFlags;
};


#endif /* __I2CSLAVE_RK_H */
