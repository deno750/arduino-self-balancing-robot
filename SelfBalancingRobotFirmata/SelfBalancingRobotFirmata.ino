#include <Firmata.h>
#include "PID_v1.h"
#include "LMotorController.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//The min abs speed
#define MIN_ABS_SPEED 20

#define KP 0x00
#define KD 0x01
#define KI 0x02
#define MOTOR_SPEED_FACTOR_LEFT 0x03
#define MOTOR_SPEED_FACTOR_RIGHT 0x04

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 175.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1; //TODO: This values must be setted on snap4Arduino
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 50; //TODO: This values must be setted on snap4Arduino
double Kd = 1.4; //TODO: This values must be setted on snap4Arduino
double Ki = 60; //TODO: This values must be setted on snap4Arduino
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6; //TODO: This values must be setted on snap4Arduino
double motorSpeedFactorRight = 0.5; //TODO: This values must be setted on snap4Arduino
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

int gyroOffests[4];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

//Callback functions that is used to setup values from the host computer.
void sysexCallback(byte command, byte argc, byte *argv) {
    double value = *((double*) argv);
    //Firmata.sendString(F(sprintf("Setted value: %f", *value)));
    switch (command)
    {
    case KP:
        Kp = value;
        break;
    case KD:
        Kd = value;
        break;
    case KI:
        Ki = value;
        break;
    case MOTOR_SPEED_FACTOR_LEFT:
        motorSpeedFactorLeft = value;
        
    case MOTOR_SPEED_FACTOR_RIGHT:
        motorSpeedFactorRight = value;
        
        break;
    default:
        break;
    }
}

void gyroOffsetsCallback(int pin, int value) {

}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Firmata initialization
    Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);
    Firmata.attach(START_SYSEX, sysexCallback); //Host computer sends the values trough ANALOG_MESSAGE channel. TODO:
    //If possible, create a custom comunication channel between host and firmata.
    Firmata.begin(115200);

    // initialize device
    Firmata.sendString("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Firmata.sendString("Testing device connections...");
    Firmata.sendString(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // load and configure the DMP
    Firmata.sendString("Initializing DMP...");
    devStatus = mpu.dmpInitialize();

    while (Firmata.available()) { //A loop that waits the firmata input.
        Firmata.processInput();
    }
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220); //TODO: This values must be setted on snap4Arduino
    mpu.setYGyroOffset(76); //TODO: This values must be setted on snap4Arduino
    mpu.setZGyroOffset(-85); //TODO: This values must be setted on snap4Arduino
    mpu.setZAccelOffset(1788); //TODO: This values must be setted on snap4Arduino

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Firmata.sendString("Enabling DMP...");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Firmata.sendString("Enabling interrupt detection (Arduino external interrupt 0)...");
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Firmata.sendString("DMP ready! Waiting for first interrupt...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        char* errorType;
        if (devStatus == 1) 
          errorType = "Initial memory load failed";
        else if (devStatus == 2)
          errorType = "DMP configuration updates failed";
        else 
          errorType = "";
        Firmata.sendString(strcat("DMP Initialization failed: ", errorType ));
    }
}

void loop() {
  while (Firmata.available()) {
    Firmata.processInput();
  }

  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors
        
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Firmata.sendString("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
        input = ypr[1] * 180/M_PI + 180;
   }
}
