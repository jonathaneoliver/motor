# I2CSlaveRK

*Class to make using a Particle Photon, Electron, etc. as an I2C slave device easier*

## Overview

These classes make it easy to make a Photon/Electron be an I2C slave. Normally the Photon/Electron is the master, communicating with sensors, displays, etc.. Using the Photon as an I2C slave allows it to be the "sensor" for another Photon or Electron, or something like a Raspberry Pi.

The model is similar to how many sensor chips work. The slave Photon is configured to have any number of 32-bit registers. Either the master or the slave can set or get these register values, but the values are stored on the I2C slave.

If the master wants to read the register value, it asks the slave for the value. If it writes a register, it sends the new value over I2C.

Additionally, the slave has a feature where it can query if a register has been set by the master. You can use this from loop when you want to use I2C to trigger something in the I2C slave, instead of merely sending values back and forth.

Since I2C is a bus, you can connect multiple I2C slave devices to the I2C master.


## I2C Master

The I2C master typically includes this header and global variable:

```
#include "I2CSlaveMasterRK.h"

I2CSlaveMaster device(Wire, 0x10);
```

You typically do something like this for setup():

```
void setup() {
	Serial.begin(9600);
	device.begin();
}
```

To read a register you might do something like this:

```
		uint32_t reg0;
		if (!device.readRegister(0, reg0)) {
			Serial.println("read register 0 failed");
			return;
		}
```

To write a register you might use something like this:

```
		int r = analogRead(A0);
		Serial.printlnf("setting 2 to %u", r);
		if (!device.writeRegister(2, r)) {
			Serial.println("write register 2 failed");
			return;
		}
```

## Slave

The I2C slave Photon/Electron typically

```

#include "I2CSlaveRK.h"

I2CSlave device(Wire, 0x10, 10);
```

You typically do something like this for setup():

```
void setup() {
	Serial.begin(9600);
	device.begin();
}
```

To set a register to be read by the I2C master you do:

```
		Serial.printlnf("register 0 set to %u", counter);
		device.setRegister(0, counter++);
```

And from loop you can find out if the master has recently set a value (even if it set the value to the same value as before) by doing:

```
	uint16_t regAddr;
	while(device.getRegisterSet(regAddr)) {
		// regAddr was updated from the I2C master
		Serial.printlnf("master updated %u to %u", regAddr, device.getRegister(regAddr));
	}
```


