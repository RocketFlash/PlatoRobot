#include <SoftwareSerial.h>

//#include <EEPROM.h>

#include <MotorWheel.h>
#include <R2WD.h>


#include <fuzzy_table.h>
#include <PID_Beta6.h>

//#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>

#include <SONAR.h>



//         - - - - - - - - - - - - - - - - 
//        /       power switch            \
//       /                                 \
//      |                                   |
//      |                                   |
//      |  wheel2                    wheel1 |
//      |                                   |
//      |                                   |
//      \                                   /
//       \  sonar3    sonar 2    sonar 1   /
//         \                              /
//           \                           /
//             -  -  -  -  -  -  -  -  -
//         bumper_R  bumper_C  bumper_L
//
//
//
//
//
//
//
/******************************************/
// SONAR

SONAR sonar11(0x11),sonar12(0x12),sonar13(0x13);

unsigned short distBuf[3];
void sonarsUpdate() {
    static unsigned char sonarCurr=1;
    if(sonarCurr==3) sonarCurr=1;
    else ++sonarCurr;
    if(sonarCurr==1) {        
        distBuf[1]=sonar12.getDist();        
        sonar12.trigger();        
    } else if(sonarCurr==2) {
        distBuf[2]=sonar13.getDist();
        sonar13.trigger();
    } else {
        distBuf[0]=sonar11.getDist();
        sonar11.trigger();
    }
}

/*********************************************/
// Optional device
// Infrared Sensor
/*
unsigned char irL0_pin=0;    // Analog
unsigned char irC0_pin=1;
unsigned char irR0_pin=2;

int ir_distance(unsigned char ir) {
	int val=analogRead(ir);
	return (6762/(val-9))-4;
}
 */
/*********************************************/
// bumper
unsigned char bumperL_pin=12;
unsigned char bumperC_pin=3;
unsigned char bumperR_pin=2;


/*********************************************/

irqISR(irq1,isr1);
MotorWheel wheel1(9,8,4,5,&irq1,REDUCTION_RATIO,int(144*PI));

irqISR(irq2,isr2);
MotorWheel wheel2(10,11,6,7,&irq2,REDUCTION_RATIO,int(144*PI));
//MotorWheel wheel2(3,2,6,7,&irq2,REDUCTION_RATIO,int(144*PI));

R2WD _2WD(&wheel1,&wheel2,WHEELSPAN);
unsigned int speedMMPS=80;

void setup() {
    //TCCR0B=TCCR0B&0xf8|0x01;    // warning!! it will change millis()
    TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
    //TCCR2B=TCCR2B&0xf8|0x01;    // Pin3,Pin11 PWM 31250Hz
    Serial.begin(9600);
    SONAR::init(13);    // Pin13 as RW Control
    
    _2WD.PIDEnable(0.35,0.02,0,10);
}

/*
void loop() {
    _2WD.demoActions(80,5000);
}
 */


void loop() {
        boolean bumperL=!digitalRead(bumperL_pin);
        boolean bumperC=!digitalRead(bumperC_pin);
        boolean bumperR=!digitalRead(bumperR_pin);
        
	//int irL0=ir_distance(irL0_pin);
	//int irC0=ir_distance(irC0_pin);
        //int irR0=ir_distance(irR0_pin);      
         
        static unsigned long currMillis=0;
        if(millis()-currMillis>SONAR::duration) {
            currMillis=millis();
            sonarsUpdate();   
        }
                 
        if(bumperL || bumperC || bumperR) {
          _2WD.setCarBackoff(speedMMPS);
          _2WD.delayMS(300);
          if(bumperL || bumperC) _2WD.setCarRotateRight(speedMMPS); // // back off and turn right
          else _2WD.setCarRotateLeft(speedMMPS);      // back off and turn left
          _2WD.delayMS(300);
        //} else if(0<irL0 && irL0<30 || 0<irC0 && irC0<40 || 0<distBuf[0] && distBuf[0]<30 || 0<distBuf[1] && distBuf[1]<40) {
        } else if(0<distBuf[0] && distBuf[0]<30 || 0<distBuf[1] && distBuf[1]<40) {  
          _2WD.setCarRotateRight(speedMMPS); 
        //} else if(0<irR0 && irR0<30 || 0<distBuf[2] && distBuf[2]<30) {
        } else if(0<distBuf[2] && distBuf[2]<30) {
          _2WD.setCarRotateLeft(speedMMPS);
        } else { 
          _2WD.setCarAdvance(speedMMPS);
        }
        _2WD.PIDRegulate();
        if(millis()%100==0) Serial.println(distBuf[2]);
//        Serial.println(_2WD.getCarStat());
        //_2WD.demoActions();
}

 
