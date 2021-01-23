/*
 * Test FlySky IBus interface on an Arduino Mega.
 *  Connect FS-iA6B receiver to Serial1.
 */

#include "FlySkyIBus.h"

void setup()
{
  Serial.begin(115200);
  IBus.begin(Serial1);
}

void loop()
{
  IBus.loop();
  Serial.printlnf("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", IBus.readChannel(0),IBus.readChannel(1),IBus.readChannel(2),IBus.readChannel(3),IBus.readChannel(4),IBus.readChannel(5),IBus.readChannel(6),IBus.readChannel(7),IBus.readChannel(8),IBus.readChannel(9) );
}
