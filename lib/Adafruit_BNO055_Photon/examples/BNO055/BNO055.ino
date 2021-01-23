/*************************

   Wiring:

   BNO055 -> Particle
   VIN -> VIN
   GND -> GND
   SDA -> D0
   SCL -> D1

   Ported to Particle devices by Nathan Robinson
   https://github.com/nrobinson2000
   On 2017-1-21

*************************/

#include "Particle.h"
#include "OneWire.h"
#include "Adafruit_BNO055_Photon.h"

//SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup()
{
        Serial.begin(9600);
        Serial.println("Orientation Sensor Test");
        Serial.println("");

        /* Initialize the sensor */
        if(!bno.begin())
        {
                /* There was a problem detecting the BNO055 ... check your connections */
                Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
                while(1) ;
        }

        delay(1000);

        bno.setExtCrystalUse(true);
}

void loop()
{
        /* Get a new sensor event */
        sensors_event_t event;
        bno.getEvent(&event);

        /* Display the floating point data */
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);
        Serial.println("");

        /* Display calibration status for each sensor. */
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIBRATION: Sys=");
        Serial.print(system, DEC);
        Serial.print(" Gyro=");
        Serial.print(gyro, DEC);
        Serial.print(" Accel=");
        Serial.print(accel, DEC);
        Serial.print(" Mag=");
        Serial.println(mag, DEC);

        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        /* Display the floating point data */
        Serial.print("X: ");
        Serial.print(euler.x());
        Serial.print(" Y: ");
        Serial.print(euler.y());
        Serial.print(" Z: ");
        Serial.print(euler.z());
        Serial.print("\t\t");
        Serial.println();
        Serial.println();

        delay(500);
}
