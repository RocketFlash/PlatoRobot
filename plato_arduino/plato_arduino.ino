
#include <TroykaIMU.h>
#include <SONAR.h>
#include <MySoftwareSerial.h>
#include <Timer.h>
#include <Wire.h>

#include <MotorWheel.h>
#include <R2WD.h>
#include <fuzzy_table.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>

// filter multiplier
#define BETA 0.2

void sonarsUpdate();
void publish_sonar();
void publish_imu();
void publish_bumpers();
void read_velocities();
void parse_data();
void update_velocities();

// Madgwick filter object
Madgwick filter;

Gyroscope gyro;
Accelerometer accel;
Compass compass;
Barometer barometer;

unsigned char bumperL_pin=12;
unsigned char bumperC_pin=3;
unsigned char bumperR_pin=2;

const double compassCalibrationBias[3] = {
  524.21,
  3352.214,
  -1402.236
};

const double compassCalibrationMatrix[3][3] = {
  {1.757, 0.04, -0.028},
  {0.008, 1.767, -0.016},
  {-0.018, 0.077, 1.782}
};

Timer timer;

unsigned short distBuf[3];
SONAR sonar11(0x11),sonar12(0x12),sonar13(0x13);

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,4,5,&irq1,REDUCTION_RATIO,int(144*PI));

irqISR(irq2,isr2);
MotorWheel wheel2(10,11,6,7,&irq2,REDUCTION_RATIO,int(144*PI));

String sended_string = "";
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
int parsed_wheel_speed_left = 0;
int parsed_wheel_speed_right = 0;
int real_wheel_speed_left = 0;
int real_wheel_speed_right = 0;

// frequency of selecting data for the filter
float fps = 100;
// variables for data from gyro, accelerometer and compass
float gx, gy, gz, ax, ay, az, mx, my, mz, agx, agy, agz;
// orientation data in quaternions
float q0, q1, q2, q3;

void setup() {
    Serial.begin(115200);
    TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz

    // IMU constituents initialization
    gyro.begin();
    accel.begin();
    compass.begin();
    barometer.begin();

    // Compass calibration
    compass.calibrateMatrix(compassCalibrationMatrix, compassCalibrationBias);
    SONAR::init(13);    // Pin13 as RW Control

    wheel1.PIDEnable(1.3,0.07,0.1,10);
    wheel2.PIDEnable(1.3,0.07,0.1,10);
    // Publish sonar and IMU data every 10 milliseconds
    //timer.every(10,publish_sonar);
    timer.every(10,publish_imu);
    sended_string.reserve(200);
}

boolean bumperL;
boolean bumperC;
boolean bumperR;
float angular_velocity_X;
float angular_velocity_Y;
float angular_velocity_Z;
float acceleration_X;
float acceleration_Y;
float acceleration_Z;
float azimuth;
float pressure;
float temperature;

void loop() {

      bumperL=!digitalRead(bumperL_pin);
      bumperC=!digitalRead(bumperC_pin);
      bumperR=!digitalRead(bumperR_pin);


      static unsigned long currMillis=0;
       if(millis()-currMillis>SONAR::duration) {
                currMillis=millis();
                //sonarsUpdate();
       }

       if(bumperL || bumperC || bumperR){
          publish_bumpers();
       }

       read_velocities();
       wheel1.PIDRegulate();
       wheel2.PIDRegulate();
       timer.update();
}

void publish_bumpers(){
      Serial.print("B[");
      Serial.print(int(bumperL));
      Serial.print(",");
      Serial.print(int(bumperC));
      Serial.print(",");
      Serial.print(int(bumperR));
      Serial.print("]B");
      Serial.println();
}

void publish_sonar(){
     Serial.print("S[");
     Serial.print(distBuf[0]);
     Serial.print(",");
     Serial.print(distBuf[1]);
     Serial.print(",");
     Serial.print(distBuf[2]);
     Serial.print("]S");
     Serial.println();
}

void publish_imu(){
     // remember current time
     unsigned long startMillis = millis();
     // angular velocity in radians per second relatively X,Y and Z axes
     gyro.readRadPerSecXYZ(&gx, &gy, &gz);

     // acceleration in m/sec^2 towards X,Y and Z axes
     accel.readAXYZ(&ax, &ay, &az);
     // acceleration in G towards X,Y and Z axes
     accel.readGXYZ(&agx, &agy, &agz);

     // compass data relatively X,Y and Z axes in Gauss
     compass.readCalibrateGaussXYZ(&mx, &my, &mz);

     // set coefficients for the filter
     filter.setKoeff(fps, BETA);
     // update filter data
     filter.update(gx, gy, gz, agx, agy, agz);
     filter.readQuaternions(&q0, &q1, &q2, &q3);

     // Absolute pressure and temperature
     //pressure = barometer.readPressureMillibars();
     //temperature = barometer.readTemperatureC();
     Serial.print("I[");
     Serial.print(gx);
     Serial.print(",");
     Serial.print(gy);
     Serial.print(",");
     Serial.print(gz);
     Serial.print(",");
     Serial.print(ax);
     Serial.print(",");
     Serial.print(ay);
     Serial.print(",");
     Serial.print(az);
     Serial.print(",");
     Serial.print(q0);
     Serial.print(",");
     Serial.print(q1);
     Serial.print(",");
     Serial.print(q2);
     Serial.print(",");
     Serial.print(q3);
     Serial.print("]I");
     Serial.println();


     // calculate processing time
     unsigned long deltaMillis = millis() - startMillis;
     // calculate filter processing frequency
     fps = 1000 / deltaMillis;
}


void sonarsUpdate() {
    static unsigned char sonarCurr=1;
    if(sonarCurr==3) sonarCurr=1;
    else ++sonarCurr;
    if(sonarCurr==1) {
        //distBuf[1]=sonar12.getDist();
        sonar12.trigger();
    } else if(sonarCurr==2) {
        distBuf[2]=sonar13.getDist();
        sonar13.trigger();
    } else {
        distBuf[0]=sonar11.getDist();
        sonar11.trigger();
    }
}

void read_velocities() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    if(Serial.available() > 0){
      if(Serial.read()== startMarker){
        sended_string = Serial.readStringUntil(endMarker);
        sended_string.toCharArray(receivedChars,sended_string.length()+1);
        //Serial.println(sended_string);
        strcpy(tempChars, receivedChars);
        parse_data();
      }else{
        Serial.flush();
      }
    }
}

void parse_data() {

      char * strtokIndx; // this is used by strtok() as an index

      strtokIndx = strtok(tempChars, ","); // this continues where the previous call left off
      parsed_wheel_speed_left = atoi(strtokIndx);     // convert this part to an integer

      strtokIndx = strtok(NULL, ",");
      parsed_wheel_speed_right = -atoi(strtokIndx);     // convert this part to a float
      update_velocities();

}

void update_velocities() {
    wheel1.setSpeedMMPS(parsed_wheel_speed_left);
    wheel2.setSpeedMMPS(parsed_wheel_speed_right);
    Serial.print("W[");
    Serial.print(wheel1.getSpeedMMPS(),DEC);
    Serial.print(",");
    Serial.print(wheel2.getSpeedMMPS(),DEC);
    Serial.print("]W");
    Serial.println();
}

void reset_position(){
  wheel1.resetCurrPulse();
  wheel2.resetCurrPulse();
}
