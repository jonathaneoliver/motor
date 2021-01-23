/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/home/joliver/photon3/motor4/src/motor4.ino"

/******************************************

Included libraries

******************************************/


//#include "Particle.h"
#include <stdarg.h>

#include <application.h>
#include "Adafruit-MotorShield-V2.h"

#include "Encoder.h"

#include "pid.h"
#include "PID-AutoTune.h"

#include "FlySkyIBus.h"

double adjustSpeed( double speed, double increment );
double balanceCorrection( double input );
void SleepStateToggle();
void SleepStateCheck(void);
void SleepStateInit();
void setup();
int setPwm( Adafruit_DCMotor *motor, int power );
int setTargetSpeed( int speed );
void loop();
#line 22 "/home/joliver/photon3/motor4/src/motor4.ino"
#if 1
#define VERBOSE( XX ) XX
#else
#define VERBOSE( XX )
#endif

typedef enum {
  IDLE=0,
  DRIVE=1,
  BALANCE_INIT=2,
  BALANCE_ACTIVE=3,
  AUTOTUNE=7,
  MQTT_TEST=8,
} balanceMode_t;

static balanceMode_t mode=IDLE;

/******************************************

Variables

******************************************/

// 
double leftPower=0.0;
double rightPower=0.0;

double adjustSpeed( double speed, double increment )
{
  double new_speed=max(min((speed+increment), 255), -255); 
  //double new_speed=increment; 
  return new_speed;
}



//Define Variables we'll be connecting to
// with aTuenStep=255 AND aTuenStartValue=0
//[loop] 67002 - left pid AUTOTUNE COMPLETE Kp=0.254455, Ki=0.008519, Kd=0.000000
//[loop] 21028 - left pid AUTOTUNE COMPLETE Kp=0.264674, Ki=0.627685, Kd=0.000000
//[loop] 47654 - left pid AUTOTUNE COMPLETE Kp=0.256071, Ki=0.085930, Kd=0.000000
//[loop] 40663 - left pid AUTOTUNE COMPLETE Kp=0.256429, Ki=0.060562, Kd=0.000000

// input Trigger line A = 362*2 =724, Pu=965-450ms=515ms, 
// output D=200
// PI -> Ku=4*D/(A*pi)= 0.3519
// Kp= 0.4*Ku = 0.14,  Ki=0.48*Ku/Pu = 0.328

//const double kp=0.256, ki=0.10, kd=0.0;
const double kp=0.356/10, ki=0.0, kd=0.0;
double leftSetpoint, leftInput, leftOutput;
double rightSetpoint, rightInput, rightOutput;

//Specify the links and initial tuning parameters
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, kp, ki, kd, PID::DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, kp, ki, kd, PID::DIRECT);

//double outputStart=5;
double aTuneStep=200, aTuneNoise=20, aTuneStartValue=0;
unsigned int aTuneLookBack=20;
boolean tuning = false;

PID_ATune aTune(&leftInput, &leftOutput);

#define BALANCE_THRESHOLD_START (3)
#define BALANCE_THRESHOLD_MAX (25)

//  balancing PID
//const double bkp=4.0, bki=0.5, bkd=0;
 double bkp=4.0, bki=0.0, bkd=0.0;
double balanceSetpoint, balanceInput, balanceOutput;
PID balancePID(&balanceInput, &balanceOutput, &balanceSetpoint, bkp, bki, bkd, PID::REVERSE);
//const double balanceOffset=108.5;
const double balanceOffset=0;

double balanceCorrection( double input )
{
  return input+balanceOffset;
}



// Change these two numbers to the pins connected to your encoder.12
// Both pins must have interrupt capability
Encoder rightEnc(D8, D6);
//Encoder rightEnc(D8, D6);
Encoder leftEnc(D4, D5);
//   avoid using pins with LEDs attached

//#define RX_ENABLE D6

#define SPEED_TINY_DELTA (10)
#define SPEED_LARGE_DELTA (100)
#define SPEED_DELTA (SPEED_LARGE_DELTA)
#define SPEED_MAX (500)
#define SPEED_MIN (-500)

long oldLeftPosition  = 0;
long oldRightPosition  = 0;
int oldLeftTime  = 0;
int oldRightTime  = 0;
int speed=0;

Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = shield.getMotor(3);
Adafruit_DCMotor *rightMotor = shield.getMotor(4);

#include "Particle.h"
#include "OneWire.h"
#include "Adafruit_BNO055_Photon.h"

#include "I2CSlaveRK.h"

//SYSTEM_MODE(MANUAL);
//SYSTEM_THREAD(ENABLED);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long serialTime; //this will help us know when to talk with processing

I2CSlave device(Wire1, 0x10, 10);


//#define MQTT_ENABLE 1

uint32_t imu_last_update=0;


#ifdef MQTT_ENABLE
#include "MQTT.h"

void MQTTcallback(char* topic, byte* payload, unsigned int length);

/**
 * if want to use IP address,
 * byte server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 * want to use domain name,
 * exp) iot.eclipse.org is Eclipse Open MQTT Broker: https://iot.eclipse.org/getting-started
 * MQTT client("iot.eclipse.org", 1883, callback);
 **/
byte MQTTserver[] = { 192,168,1,135 };//the IP of broker

MQTT MQTTclient(MQTTserver, 1883, 60, MQTTcallback, 512);

// receive message
void MQTTcallback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = 0;
    Serial.printlnf("[%s]%.*s",__FUNCTION__,length, payload);

    if (!strcmp(p, "RED"))
        RGB.color(255, 0, 0);
    else if (!strcmp(p, "GREEN"))
        RGB.color(0, 255, 0);
    else if (!strcmp(p, "BLUE"))
        RGB.color(0, 0, 255);
    else
        RGB.color(255, 255, 255);
    delay(100);
}
#endif

/******************************************
void setup()

Runs once upon startup
******************************************/
typedef enum {
  AWAKE,
  ASLEEP
} sleepState_t;

static sleepState_t sleepState=AWAKE;
static int sleepStateTime=0;

#define SLEEPSTATE_PIN A0

const char * SleepStateStr( sleepState_t sleepState )
{
  switch (sleepState) 
  {
    case AWAKE:
      return "AWAKE";
      break;
    case ASLEEP:
      return "ASLEEP";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}




void SleepStateToggle()
{
  // debounce
  if ( millis()-sleepStateTime < 1000 )
  {
    // ignore this
    VERBOSE(  Serial.printlnf( "[%s] %lu -> debounce ", __FUNCTION__, millis()) ) ;
    return;
  }

  sleepStateTime=millis();
  

  if ( sleepState==AWAKE )
  {

    VERBOSE( Serial.printlnf( "[%s] %lu enter sleep mode for 60 seconds or A0 falling", __FUNCTION__, millis() ) );

    delay(100);
    sleepState=ASLEEP;
    System.sleep(SLEEPSTATE_PIN, FALLING, 60 );
  } 
  else
  {
    /* this is the default when woken from System.sleep() by timer or interrupt from pin */
    sleepState=AWAKE;
  }

  VERBOSE(  Serial.printlnf( "[%s] %lu ->%s ", __FUNCTION__, millis(), SleepStateStr(sleepState)) ) ;
}

void SleepStateCheck(void)
{
  if ( digitalRead(SLEEPSTATE_PIN)==LOW )
  {
    VERBOSE(  Serial.printlnf( "[%s] %lu sleepState=%s (%ld) ", __FUNCTION__, millis(), SleepStateStr(sleepState), digitalRead(SLEEPSTATE_PIN) ) );
    SleepStateToggle();  
  }
}

void SleepStateInit()
{
  VERBOSE( Serial.printlnf( "[%s]", __FUNCTION__ ) );
  pinMode(SLEEPSTATE_PIN, INPUT_PULLUP);    // sets pin as input - high by default
}



void setup() {
  Serial.begin(115200);

  Serial.printlnf("setup()\n");
  


  //Serial.printlnf("[%s]wating for wifi",__FUNCTION__);    
  //waitUntil(WiFi.ready);
  //Serial.printlnf("[%s]wating for wifi ready",__FUNCTION__);    

#if 1
  for (;!bno.begin();)
  //if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(1000);
  }
  //bno.setExtCrystalUse(true);
#endif
  Serial.printlnf("[%s]bno ready",__FUNCTION__);    

  delay(100);

  RGB.control(true);
  Serial.printlnf("[%s]rgb ready",__FUNCTION__);    

#ifdef MQTT_ENABLE
  // connect to the server
  MQTTclient.connect(System.deviceID());
  Serial.printlnf("[%s]mqtt ready",__FUNCTION__);    

  // publish/subscribe
  if (MQTTclient.isConnected()) {
    MQTTclient.subscribe("color");//color is the topic that photon is subscribed
    MQTTclient.publish("fun", "hello");//publishing a data "hello" to the topic "fun"
    Serial.printlnf("[%s] publish hello",__FUNCTION__);
  }
  else
  {
    Serial.printlnf("[%s] MQTT FAILED to connect",__FUNCTION__);    
  }
#endif


	device.begin();
  Serial.printlnf("[%s]i2c slave ready",__FUNCTION__);    

  SleepStateInit();
  Serial.printlnf("[%s]low power ready",__FUNCTION__);    

  IBus.begin(Serial1);
  Serial.printlnf("[%s]ibus ready",__FUNCTION__);    

  Serial.printlnf(__FUNCTION__);
  
  shield.begin();

  balanceSetpoint=0;
  balancePID.SetMode(PID::AUTOMATIC);
  balancePID.SetOutputLimits(-255, 255);
  balancePID.SetSampleTime(20);

  leftSetpoint=0;
  rightSetpoint=0;

  speed=0;
  leftPower=setPwm( leftMotor, 0);
  rightPower=setPwm( rightMotor, 0);

  leftPID.SetMode(PID::AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  leftPID.SetSampleTime(10);
  //leftPID.SetSampleTime(100);

  rightPID.SetMode(PID::AUTOMATIC);
  rightPID.SetOutputLimits(-255, 255);
  rightPID.SetSampleTime(10);
//  rightPID.SetSampleTime(100);
  Serial.printlnf("[%s]pids ready",__FUNCTION__);    
  
  Serial.printlnf("setup( complete )\n");
  RGB.color(0, 255, 0);
}

/******************************************
void loop()

Runs forever
******************************************/
int setPwm( Adafruit_DCMotor *motor, int power )
{  
  if( power==0 ) {
    motor->setSpeed(abs(power));
#if 1
    motor->run(FORWARD);
#else
    motor->run(RELEASE);
#endif
  }
  else if( power>0 ) {
    motor->run(FORWARD);
    motor->setSpeed(abs(power));
  }
  else
  {
    motor->run(BACKWARD);
    motor->setSpeed(abs(power));      
  }
  return power;
}

int setTargetSpeed( int speed )
{  
  VERBOSE(  Serial.printlnf( "[%s] %lu -> %d  ", __FUNCTION__, millis(), speed) ) ;
  leftSetpoint=rightSetpoint=speed;
  return speed;
}

void telemetrySend( const char *name, const char *format, ...)
{
  char buffer[256];
  va_list args;
  va_start (args, format);
  vsnprintf (buffer, sizeof(buffer), format, args);
  va_end (args);

  VERBOSE(  Serial.printlnf( buffer ) ) ;


#ifdef MQTT_ENABLE
      if (MQTTclient.isConnected())
      {
#if 1
        MQTTclient.publish( name, buffer );
#endif
      }
      else 
      {
        Serial.printlnf("[%lu]client failed to connect",Time.now());
        // connect to the server
        MQTTclient.connect(System.deviceID());
        Serial.printlnf("[%s]mqtt ready",__FUNCTION__);    

        // publish/subscribe
        if (MQTTclient.isConnected()) {
          MQTTclient.subscribe("color");//color is the topic that photon is subscribed
          MQTTclient.publish("fun", "hello");//publishing a data "hello" to the topic "fun"
          Serial.printlnf("[%lu] publish hello",Time.now());
        }
        else
        {
          Serial.printlnf("[%lu] MQTT FAILED to connect",Time.now());    
        }
      }
#endif
#if 0
      {
        Serial.printlnf(buffer);
      }
#endif 
}



void loop() 
{
  static bool TxOn=false;

  SleepStateCheck();

	uint16_t regAddr;
	while(device.getRegisterSet(regAddr)) {
		// regAddr was updated from the I2C master
		Serial.printlnf("master updated %u to %lu", regAddr, device.getRegister(regAddr));
	}





#ifdef MQTT_ENABLE
  if (MQTTclient.isConnected())
  {
    MQTTclient.loop();
  }
  else
  {
    Serial.printlnf("[%s]client failed to connect",__FUNCTION__);
  #ifdef MQTT_ENABLE
    // connect to the server
    MQTTclient.connect(System.deviceID());
    Serial.printlnf("[%s]mqtt ready",__FUNCTION__);    

    // publish/subscribe
    if (MQTTclient.isConnected()) {
      MQTTclient.subscribe("color");//color is the topic that photon is subscribed
      MQTTclient.publish("fun", "hello");//publishing a data "hello" to the topic "fun"
      Serial.printlnf("[%s] publish hello",__FUNCTION__);
    }
    else
    {
      Serial.printlnf("[%s] MQTT FAILED to connect",__FUNCTION__);    
    }
#endif
  
  }
#endif


  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
#if 1
  static double prevZ=0.0;

  if ( prevZ != event.orientation.z )
  {
    /* Display the floating point data */
#if 0
   Serial.printlnf("(%d) X: %f\tY: %f\tZ: %f", millis(), event.orientation.x, event.orientation.y, event.orientation.z );

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.printlnf("CALIBRATION: Sys=%d\tGyro=%d\tAccel=%d\tMag=%d", system, gyro, accel, mag );
    imu::Vector<3> euler = bno.getVector(Adaf ruit_BNO055::VECTOR_EULER);
    Serial.printlnf("X: %f\tY: %f\tZ: %f", euler.x(), euler.y(), euler.z() );
#else
    VERBOSE( Serial.printlnf("(%lu) Z: %f", millis(), event.orientation.z ) );

    //VERBOSE( Serial.printlnf("(%d) set Z for i2c : %f", millis(), event.orientation.z ) );
		device.setRegister(0, int(event.orientation.z*100));

    telemetrySend( "robot/imu", "{ \"timestamp\": %lu, \"millis\": %lu,  \"name\":\"imu\", \"type\": %lu, \"imutime\":%lu, \"X\": %f, \"Y\":%f, \"Z\" : %f }",Time.now(), millis(), event.type, event.timestamp, event.orientation.x, event.orientation.y, event.orientation.z );
    balanceInput=balanceCorrection( event.orientation.z );
    VERBOSE( Serial.printlnf("(%lu) balanceInput: %f", millis(), balanceInput ) );

#if 0
    imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    telemetrySend( "imu", "{ \"timestamp\": %lu, \"millis\": %lu,  \"name\":\"imu\", \"type\": \"VECTOR_LINEARACCEL\", \"X\": %f, \"Y\":%f, \"Z\" : %f }",Time.now(), millis(), la.x(), la.y(), la.z() );
#endif

#if 0
    imu::Vector<3> gs = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    //telemetrySend( "imu", "{ \"timestamp\": %lu, \"millis\": %lu,  \"name\":\"imu\", \"type\": \"VECTOR_GYROSCOPE\", \"X\": %f, \"Y\":%f, \"Z\" : %f }",Time.now(), millis(), int(gs.x()*57.2958), int(gs.y()*57.2958), int(gs.z()*57.2958) );
    telemetrySend( "robot/imu", "{ \"timestamp\": %lu, \"millis\": %lu,  \"name\":\"imu\", \"type\": \"VECTOR_GYROSCOPE\", \"X\": %f, \"Y\":%f, \"Z\" : %f }",Time.now(), millis(), gs.x()*57.2958, gs.y()*57.2958, gs.z()*57.2958 );
#endif

    prevZ = event.orientation.z;
#endif
  }


#endif

  //if ( digitalRead(RX_ENABLE) ) 
  { 
    IBus.loop();

    uint32_t imu_update=IBus.lastUpdate();

    if ( imu_update!=imu_last_update)
    {
      // TX ON ( top left switch )
      //telemetrySend( "iBus", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\":\"iBus\", \"name\":\"iBus\", \"Channels\" : [ %d,%d,%d,%d,%d,%d,%d,%d,%d,%d ] }", Time.now(), millis(), IBus.readChannel(0),IBus.readChannel(1),IBus.readChannel(2),IBus.readChannel(3),IBus.readChannel(4),IBus.readChannel(5),IBus.readChannel(6),IBus.readChannel(7),IBus.readChannel(8),IBus.readChannel(9) );
      telemetrySend( "robot/iBus", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\":\"iBus\", \"name\":\"iBus\", \"C0\":%d,\"C1\":%d,\"C2\":%d,\"C3\":%d,\"C4\":%d,\"C5\":%d,\"C6\":%d,\"C7\":%d,\"C8\":%d,\"C9\":%d }", Time.now(), millis(), IBus.readChannel(0),IBus.readChannel(1),IBus.readChannel(2),IBus.readChannel(3),IBus.readChannel(4),IBus.readChannel(5),IBus.readChannel(6),IBus.readChannel(7),IBus.readChannel(8),IBus.readChannel(9) );
      imu_last_update=imu_update;
    }

    switch ( IBus.readChannel(7) ) {
      default:
      case 1000:
        if( !(mode!=BALANCE_INIT) && (mode!=BALANCE_ACTIVE) )
        {
          mode=IDLE;
          Serial.printlnf("STOP balanceInput from RX = %f", balanceInput );
        }      
        break;
      case 1500:
        if( (mode!=BALANCE_INIT) && (mode!=BALANCE_ACTIVE) )
        {
          mode=BALANCE_INIT;
          Serial.printlnf("START balanceInput from RX = %f", balanceInput );
        }
        break;
      case  2000: 
        break;
    }

    //{
    //  // OK use RC input for balance target angle
    //  balanceInput=(90.0*(IBus.readChannel(1)-0x5dc))/500.0;
    //  Serial.printlnf("balanceInput from RX = %f", balanceInput );
    //}
        
    static double prevLeftSetpoint=-1;
    static double prevRightSetpoint=-1;  

    switch(IBus.readChannel(6)) {
      default:
      case 1000:
        if( TxOn==true )
        {
          speed=setTargetSpeed(0);
          mode=IDLE;
          TxOn=false;
          RGB.color(0, 255, 0);
        }
        break;

      case 1500:
        // arcade drive
        if ( TxOn==false) {
          TxOn=true;
          mode=DRIVE;
          prevLeftSetpoint=-1;
          prevRightSetpoint=-1;
          //RGB.color(255, 0, 0);
          RGB.color(0, 0, 255);
        }
        else{
          //left_motor_speed =  y + x
          //right_motor_speed = y - x

          // right
          static int prevSteering=-1;
          static int prevThrottle=-1;

          int steering=IBus.readChannel(0)-1500;
          int throttle=IBus.readChannel(2)-1500;
          if ( steering!=prevSteering || throttle!=prevThrottle )
          {
            Serial.printlnf("throttle=%d, steering=%d", throttle, steering );
            prevThrottle=throttle;
            prevSteering=steering;
          }
          rightSetpoint=max( min( throttle-steering, SPEED_MAX ), SPEED_MIN );
          if ( rightSetpoint != prevRightSetpoint ) {
            Serial.printlnf("rightSetpoint from RX = %f", rightSetpoint );
            prevRightSetpoint=rightSetpoint;
          }

          // left
          leftSetpoint=(double(max( min( throttle+steering, SPEED_MAX ), SPEED_MIN )));
          if ( leftSetpoint != prevLeftSetpoint ){
            Serial.printlnf("leftSetpoint from RX = %f", leftSetpoint );
            prevLeftSetpoint=leftSetpoint;
          }

        }
        break;
      case 2000:
        // tank drive
        if ( TxOn==false) {
          TxOn=true;
          mode=DRIVE;
          prevLeftSetpoint=-1;
          prevRightSetpoint=-1;
          RGB.color(255, 0, 0);
        }
        else{
          // right
          rightSetpoint=max( min( IBus.readChannel(1)-1500, SPEED_MAX ), SPEED_MIN );
          if ( rightSetpoint != prevRightSetpoint ) {
            Serial.printlnf("rightSetpoint from RX = %f", rightSetpoint );
            prevRightSetpoint=rightSetpoint;
          }

          // left
          leftSetpoint=(double(max( min( IBus.readChannel(2)-1500, SPEED_MAX ), SPEED_MIN )));
          if ( leftSetpoint != prevLeftSetpoint ){
            Serial.printlnf("leftSetpoint from RX = %f", leftSetpoint );
            prevLeftSetpoint=leftSetpoint;
          }

        }
        break;
    }

    switch(IBus.readChannel(5)) {

      case 2000:
        // reset PID
        leftPID.SetMode(PID::MANUAL);
        leftOutput=0;
        leftPower=0;
        setPwm( leftMotor, leftOutput);

        rightPID.SetMode(PID::MANUAL);
        rightOutput=0;
        rightPower=0;
        setPwm( rightMotor, rightOutput);

        speed=setTargetSpeed(0);
        leftPID.SetMode(PID::AUTOMATIC);
        rightPID.SetMode(PID::AUTOMATIC);

        TxOn=false;
        mode=IDLE;
        //RGB.color(0, 255, 0);
        RGB.color(0, 255, 0);

        break;
      default:
        break;
    }
  }

  if (Serial.available())
  {
    int inByte = Serial.read();
    VERBOSE(  Serial.printlnf( "[%s] %lu -> %d  ", __FUNCTION__, millis(), inByte) ) ;
    switch (inByte)
    {
        case ' ':
          // reset PID
          leftPID.SetMode(PID::MANUAL);
          leftOutput=0;
          leftPower=0;
          setPwm( leftMotor, leftOutput);

          rightPID.SetMode(PID::MANUAL);
          rightOutput=0;
          rightPower=0;
          setPwm( rightMotor, rightOutput);

          speed=setTargetSpeed(0);
          leftPID.SetMode(PID::AUTOMATIC);
          rightPID.SetMode(PID::AUTOMATIC);

          mode=IDLE;

          break;
        case '1':

          //void SetTunings(double, double,       // * While most users will set the tunings once in the 
          //          double);         	  //   constructor, this function gives the user the option
          //                                //   of changing tunings during runtime for Adaptive control

          bkp+=1;
          balancePID.SetTunings(bkp, bki, bkd);
          VERBOSE(  Serial.printlnf( "[%s] %lu BALANCE PID BKp-> %f ", __FUNCTION__, millis(), bkp) ) ;
          break;
        case '2':
          bkp-=1;
          balancePID.SetTunings(bkp, bki, bkd);
          VERBOSE(  Serial.printlnf( "[%s] %lu BALANCE PID BKp-> %f ", __FUNCTION__, millis(), bkp) ) ;
          break;
        case '3':
          speed=setTargetSpeed(-speed);
          mode=DRIVE;
          break;
        case '5':
          VERBOSE(  Serial.printlnf( "[%s] %lu -> BALANCE_INIT  ", __FUNCTION__, millis()) ) ;
          mode=BALANCE_INIT;
          break;         
        case '6':
          VERBOSE(  Serial.printlnf( "[%s] %lu -> MQTT_TEST  ", __FUNCTION__, millis()) ) ;
          mode=MQTT_TEST;
          break;              
        case '8':
          if( mode==AUTOTUNE) {
            aTune.Cancel();
            tuning = false;
            mode=IDLE;
          }
          else
          {
            mode=AUTOTUNE;
            //Set the output to the desired starting frequency.
            leftOutput=aTuneStartValue;
            aTune.SetNoiseBand(aTuneNoise);
            aTune.SetOutputStep(aTuneStep);
            aTune.SetLookbackSec((int)aTuneLookBack);

            tuning=true;
            serialTime = 0;

          }
          break;              
        case '9':
          SleepStateToggle();
          break;
        default:
          break;
    }
  }

  long newLeftPosition = leftEnc.read();
  //VERBOSE(  Serial.printlnf( "[%s] %d -> LeftEncoder = %d  ", __FUNCTION__, millis(),newLeftPosition) ) ;

  long newRightPosition = rightEnc.read();
  //VERBOSE(  Serial.printlnf( "[%s] %d -> RightEncoder = %d  ", __FUNCTION__, millis(),newRightPosition) ) ;
  int time=millis();
  switch(mode)
  {
    case MQTT_TEST:
      VERBOSE(  Serial.printlnf( "[%s] %lu -> MQTT_TEST  ", __FUNCTION__, millis()) ) ;
      telemetrySend( "robot/test", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\": \"test\" }",Time.now(), millis() );
      break;

    case BALANCE_INIT:
      // wait for robot to be vertical then start balancing
      if ( abs(balanceInput)<BALANCE_THRESHOLD_START )
      {
        // start balancing
        mode=BALANCE_ACTIVE;
        Serial.printlnf( "balance goes active:  %lu, %f", millis(), balanceInput ) ;
      }
      break;
    case BALANCE_ACTIVE:
      if ( abs(balanceInput)>BALANCE_THRESHOLD_MAX )
      {
        // too far gone - give up - go idle
        mode=BALANCE_INIT;
        Serial.printlnf( "balance goes idle: %lu, %f", millis(), balanceInput ) ;
      }
    #if 1
      //Serial.printlnf( "[%s]%f", __FUNCTION__, leftInput );
      if ( balancePID.Compute() )
      {
        leftSetpoint=balanceOutput;
        rightSetpoint=balanceOutput;

        Serial.printlnf( "balancePid { \"timestamp\": \"%lu\", \"Input\" : \"%f\", \"Setpoint\"  : \"%f\", \"Output\" : \"%f\" }", millis(), balanceInput, balanceSetpoint, balanceOutput  ) ;

        telemetrySend( "robot/balancePid", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\":\"pid\", \"name\":\"balancePid\", \"Input\" : %f, \"Setpoint\"  : %f, \"Output\" : %f, \"Kp\":%f, \"Ki\":%f, \"Kd\":%f, \"mode\":\"%s\"  }", Time.now(), millis(), balanceInput, balanceSetpoint, balanceOutput, balancePID.GetKp(), balancePID.GetKi(), balancePID.GetKd(), (balancePID.GetMode()==PID::AUTOMATIC)?"AUTOMATIC":"MANUAL" ) ;
      }
    #endif
      // break;
    case DRIVE:
    #if 1
      if ( oldLeftTime==0 )
      {
        // do nothing
        oldLeftPosition = newLeftPosition;
        oldLeftTime = time;
      } 
      else if (time-oldLeftTime>0)
      {  
        //Serial.printlnf( "[%s]%d", __FUNCTION__, time-oldLeftTime );
        leftInput = (((newLeftPosition-oldLeftPosition)*60000)/(18.75*64*(time-oldLeftTime)));
        //Serial.printlnf( "[%s]%f", __FUNCTION__, leftInput );
        if ( leftPID.Compute() )
        {
          //telemetrySend( "robot/leftPid", "{ \"timestamp\": %lu, \"millis\":%lu, \"type\":\"pid\", \"name\":\"leftPid\", \"Input\" : %f, \"Setpoint\"  : %f, \"Output\" : %f, \"Kp\":%f, \"Ki\":%f, \"Kd\":%f, \"mode\":\"%s\"  }", Time.now(), millis(), leftInput, leftSetpoint, leftOutput, leftPID.GetKp(), leftPID.GetKi(), leftPID.GetKd(), (leftPID.GetMode()==PID::AUTOMATIC)?"AUTOMATIC":"MANUAL" ) ;

          leftPower=adjustSpeed( leftPower, leftOutput );
          setPwm( leftMotor, leftPower );
          //Serial.printlnf( "[%s]%d left power=%d, delta=%d (position=%d) RPM=%f", __FUNCTION__, Time.now(), power, newLeftPosition-oldLeftPosition, newLeftPosition, leftInput );
          VERBOSE( Serial.printlnf( "[%s] %d - left pid input=%f, setpoint=%f, output=%f", __FUNCTION__, time, leftInput, leftSetpoint, leftOutput) );
          //telemetrySend( "robot/leftPower", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\":\"power\", \"name\":\"leftPower\", \"power\" : %f  }", Time.now(), millis(), leftPower ) ;
          VERBOSE( Serial.printlnf( "[%s] %d - left power=%f", __FUNCTION__, time, leftPower) );

          oldLeftPosition = newLeftPosition;
          oldLeftTime = time;

        }
      }

    #endif

    #if 1
      if ( oldRightTime==0 )
      {
        // do nothing
        oldRightPosition = newRightPosition;
        oldRightTime = time;
      } 
      else if (time-oldRightTime>0)
      {  
        rightInput = (((newRightPosition-oldRightPosition)*60000)/(18.75*64*(time-oldRightTime)));


        if ( rightPID.Compute() )
        {
            telemetrySend( "robot/rightPid", "{ \"timestamp\": %lu, \"millis\":%lu, \"type\":\"pid\", \"name\":\"rightPid\", \"Input\" : %f, \"Setpoint\"  : %f, \"Output\" : %f, \"Kp\":%f, \"Ki\":%f, \"Kd\":%f, \"mode\":\"%s\"  }", Time.now(), millis(), rightInput, rightSetpoint, rightOutput, rightPID.GetKp(), rightPID.GetKi(), rightPID.GetKd(), (rightPID.GetMode()==PID::AUTOMATIC)?"AUTOMATIC":"MANUAL" ) ;

            rightPower=adjustSpeed( rightPower, rightOutput );
            setPwm( rightMotor, rightPower);
            //Serial.printlnf( "[%s]%d right power=%d, delta=%d (position=%d) RPM=%f", __FUNCTION__, time, power, newRightPosition-oldRightPosition, newRightPosition, rightInput);
            telemetrySend( "robot/rightPower", "{ \"timestamp\": %lu, \"millis\": %lu, \"type\":\"power\", \"name\":\"rightPower\", \"power\" : %f  }", Time.now(), millis(), rightPower ) ;
            VERBOSE( Serial.printlnf( "[%s] %d - right pid input=%f, setpoint=%f, output=%f", __FUNCTION__, time, rightInput, rightSetpoint, rightOutput) );
            VERBOSE( Serial.printlnf( "[%s] %d - right power=%f", __FUNCTION__, time, rightPower) );

            oldRightPosition = newRightPosition;
            oldRightTime = time; 
        }

      }    
    #endif      
      break; 
    case AUTOTUNE:
      if ( oldLeftTime==0 )
      {
        // do nothing
        oldLeftPosition = newLeftPosition;
        oldLeftTime = time;
      } 
      else if (time-oldLeftTime>0)
      {  
        //Serial.printlnf( "[%s]%d", __FUNCTION__, time-oldLeftTime );
        leftInput = (((newLeftPosition-oldLeftPosition)*60000)/(18.75*64*(time-oldLeftTime)));
        //Serial.printlnf( "[%s]%f", __FUNCTION__, leftInput );

        if ((aTune.Runtime())!=0)
        {
          //we're done, set the tuning parameters
          tuning = false;
          VERBOSE( Serial.printlnf( "[%s] %d - left pid AUTOTUNE COMPLETE Kp=%f, Ki=%f, Kd=%f", __FUNCTION__, time, aTune.GetKp(), aTune.GetKi(), aTune.GetKd()) );
          mode=IDLE;
          leftPower=adjustSpeed( leftPower, 0 );
          setPwm( leftMotor, leftPower );          
        }
        else
        {
          leftPower=adjustSpeed( leftPower, leftOutput );
          setPwm( leftMotor, leftPower );
          //Serial.printlnf( "[%s]%d left power=%d, delta=%d (position=%d) RPM=%f", __FUNCTION__, time, power, newLeftPosition-oldLeftPosition, newLeftPosition, leftInput );
          VERBOSE( Serial.printlnf( "[%s] %d - left pid AUTOTUNE RUNNING input=%f, setpoint=%f, output=%f", __FUNCTION__, time, leftInput, leftSetpoint, leftOutput) );
          telemetrySend( "robot/leftPower", "{ \"timestamp\": %lu, \"millis\": %lu,  \"type\":\"power\", \"name\":\"leftPower\", \"power\" : %f  }", Time.now(), millis(),leftPower ) ;
          VERBOSE( Serial.printlnf( "[%s] %d - left power=%f", __FUNCTION__, time, leftPower) );

          oldLeftPosition = newLeftPosition;
          oldLeftTime = time;
        }
      }
      break;
  }

}
