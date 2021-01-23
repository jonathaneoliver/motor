/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
// Both pins must have interrupt capability
Encoder myEnc(A6, A2);

//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Encoder Ticks Test:");
}

long oldPosition  = -999;
uint32_t oldTicks = 0;

void loop() {
  uint32_t ticks=0;
  long newPosition = myEnc.readWithTicks(&ticks);
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.printlnf("%d, %ul -> %f \n", newPosition, ticks, float(ticks-oldTicks)/System.
ticksPerMicrosecond());
    oldTicks=ticks;
  }
}
