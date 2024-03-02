//comms to memory
#include <Wire.h>
#define EEPROM_I2C_ADDRESS 0x50 //External memory chip address

/*
*ECARGO CONTROLLER
*MICHAEL JEANS
*BLDCREGEN_1.3
*HALLS VERSION
*PWM CONTROLLED SPEED/BRAKING
*DEVELOPMENT STAGE
*/

//////////////////SETUP/////////////////////////////

void setup() {

////////////PIN DATA/////////////////////////

//debug LED
pinMode(13, OUTPUT); //LED //change to pin13 for actual board
pinMode(LED_BUILTIN, OUTPUT);
  
// Halls input  
  pinMode(0,INPUT);    // Hall 1
  pinMode(1,INPUT);    // Hall 2
  pinMode(2,INPUT);    // Hall 3

// Outputs for the Motor Drivers
  pinMode(3,OUTPUT);   // IN 1
  pinMode(5,OUTPUT);   // IN 2
  pinMode(6,OUTPUT);   // IN 3    
  pinMode(9,OUTPUT);   // EN 1
  pinMode(10,OUTPUT);  // EN 2
  pinMode(11,OUTPUT);  // EN 3

// Sensor Inputs
  pinMode(A0, INPUT);   //Strain gauge
  pinMode(A1, INPUT);   //Winding Temp
  pinMode(A2, INPUT);   //Bike Speed

//Comms
  pinMode(SDA, OUTPUT); //SDA
  pinMode(SCL, OUTPUT); //SCL
  Wire.begin();         //initialize I2C busses
  Serial.begin(9600);   //To write to serial monitor for debug

///////////////////PWM AND INTERRUPT SETUP//////////////////////////

///////////////////1ms timer zero interrupt/////////////////////////

cli();//stop interrupts
  //set timer0 interrupt at 1kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 1khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
sei();//enable interrupts 

// This sets PWM for pins 9 and 10 to 31372.55 Hz //CS divisor =1
TCCR1B = TCCR1B & B11111000 | B00000001;

// This sets PWM for pins 3 and 11 to 31372.55 Hz //CS divisor =1
TCCR2B = TCCR2B & B11111000 | B00000001;
} //closesetup

//////////////////Declarations/////////////////////

///////////////////////Functions/////////////////////
void halls();
void checkspeedload();
void memory();
void temp();
void loadcalc();
void commutation();

///////////////////////Interrupt/////////////////////

ISR(TIMER0_COMPA_vect);  

/////////////Variables ////////////////////

//Hall sensors (3,2,1) + Hall value for easy calc
int Hall1; 
int Hall2;
int Hall3;
int HallVal = 1;
int NewHallVal;

//speed levels for drive and brake modes
int MotorSpeedLevel = 0; //speed level of the motor
int BrakeSpeedLevel = 0; //braking level
int throttle = 0; //mapped throttle difference to set brake and motor speed.

//Load
int loadinit = analogRead(A0); //What is the load at rest/unloaded? Make this the centre point
int load[10];
int p = 0;
int q = 0;
int aveload = 0;

//Bike Speed 
int ecargospeed = 0; //real ecargo speed in rpm
//float wheelsize = 0.0016; //circumference in km for 20inch wheel plus hefty tires
char steps= 0 ; 
char neworderpointer = 0;
char orderpointer = 0;
int speed[100]; //speed averaging filter
int n = 0; //speed array pointer

//Temp
int windingtemp;     //winding temp

//Timing variable
char m = 0;
int i = 0;

//I2C
int loadlevelspeed;
int loadlevel;

/////////////////main loop//////////////////////////////////////

//MAIN

void loop(){
  
} //main

////////////////interrupt service routine//////////////////////

ISR(TIMER0_COMPA_vect){//timer0 interrupt 1kHz
  //PORTB ^= B00100000;// toggles bit which affects pin13     //debug
halls();
commutation();
  i++;
  if(i==100){  //100ms
  //digitalWrite(13, HIGH);  //debug
  digitalWrite(LED_BUILTIN, HIGH);  //debug
    checkspeedload();   //check speed and load
    i=0;        //reset counter
  m++;
  if(m==10){ //1000ms
  //digitalWrite(13, LOW); //debug
  digitalWrite(LED_BUILTIN, LOW);  //debug
    temp();   //check temp
  //  memory(); //print to memory
    m=0;      //reset counter
    } //close1000ms
  }//close100ms
}//close isr

/////////////////FUNCTIONS/////////////////////////////////////

void halls(){     //check every 1ms
  Hall1 = digitalRead(0);  // read input value from Hall 1
  Hall2  = digitalRead(1);  // read input value from Hall 2
  Hall3  = digitalRead(2);  // read input value from Hall 3
  NewHallVal = (Hall1) + (2*Hall2) + (4*Hall3); //Computes the binary value of the 3 Hall sensors

    switch (NewHallVal){
      case 3:
        neworderpointer = 1;
      break;
      case 1:
        neworderpointer = 2;
      break;
      case 5:
        neworderpointer = 3;
      break;
      case 4:
        neworderpointer = 4;
      break;
      case 6:
        neworderpointer = 5;
      break;
      case 2:
        neworderpointer = 6;
      break;
    }//switch

  //calculate no of steps  //order = [3,1,5,4,6,2];
  if(orderpointer <= neworderpointer){
    steps = neworderpointer - orderpointer;
  }
  else{
    steps = (6-orderpointer) + neworderpointer;
  }
  orderpointer = neworderpointer;
   
  speed[n] = steps*320; //3 pole 
  n++;
  if (n > 99){
    n=0;
  }
  HallVal = NewHallVal;
  HallVal = 2; //Debug and for testing
}//halls

void checkspeedload(){    //average speed/load for check EVERY 100ms
loadcalc();
  for(n = 0; n < 100; n++){ 
  ecargospeed = 0;
  ecargospeed += speed[n];
    }
  n=0;
  ecargospeed = ecargospeed/100;
}//checkspeedload

void memory(){        //print every second
//Loadlevel
loadlevel = map(throttle, 0, 255, 0, 7);
loadlevel = loadlevel << 5;
//Create loadlevelspeed byte hi largest bits are load level, 5 lowest are speed
loadlevelspeed = 0x0000 || ecargospeed;
loadlevelspeed = loadlevelspeed || loadlevel;

//print speed/temp/load
Wire.beginTransmission(EEPROM_I2C_ADDRESS); // transmit to device
  Wire.write(windingtemp);   // sends 1 byte
  Wire.write(loadlevelspeed);   // sends 1 byte containing load and speed   
  Wire.endTransmission();    // stop transmitting
}//memory

void temp(){
//Caculate winding temperature
  windingtemp = analogRead(A1);
  windingtemp = map(windingtemp, 0, 1023, -40, 180);
}//temp

void loadcalc(){                  //include array for averaging
//Calculate load
  load[p] = analogRead(A0) - loadinit; 
  p++;
  if (p > 9){
    p=0;
  }
  for(q = 0; q < 10; q++){ 
  aveload = 0;
  aveload += load[q];
  aveload = aveload/10;
  }

//map(value, fromLow, fromHigh, toLow, toHigh)
//Changing fromHigh and fromLow values to changes the max/min load value which is at 100% Duty
throttle = map(aveload, -400, 400, 0, 255); //exceeding the init range will max out the new range
//Motor settings for braking vs driving 
MotorSpeedLevel = map(throttle, 127, 255, 0, 255); //motoring is mapped to the top half of range
BrakeSpeedLevel = map(throttle, 0, 127, 255, 0);    // regenerative braking on bottom half of range
}//end loadcalc

// Commutation for Motoring
/*
PORTD contains the outputs for the HIGHside pins
AnalogWrite command is used to set the LOWside pins
(0 = OFF, 255 = ON or motorspeedlevel value that is controlled by the load sensor)
*/
void commutation() {
  
  //debug values to set PWM to drive @200, below max speed.
  ecargospeed = 1;
  throttle = 200; 
  BrakeSpeedLevel = 200;
  MotorSpeedLevel = 127;
  //'HallVal' IS ALSO SET FOR DEBUG IN THE 'halls' FUNCTION  
  
  if(ecargospeed>25){  //this 'if' may need optimised so downhill speed is held at 25kph smoothly
    throttle = 0;
    BrakeSpeedLevel = 255;
    }
  if (throttle > 127){
      switch (HallVal)
       {
        case 3:
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B01000000;
          //Lowside
          analogWrite(9,MotorSpeedLevel); 
          analogWrite(10,0);
          analogWrite(11,0);
          break;
        case 1:
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B01000000;
          //Lowside
          analogWrite(9,0); 
          analogWrite(10,MotorSpeedLevel);
          analogWrite(11,0);
          break;
        case 5:
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B00001000;
          //Lowside
          analogWrite(9,0); 
          analogWrite(10,MotorSpeedLevel);
          analogWrite(11,0);
          break;
        case 4: 
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B00001000;
          //Lowside
          analogWrite(9,0);
          analogWrite(10,0);
          analogWrite(11,MotorSpeedLevel);
          break;
        case 6:
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B00100000;
          //Lowside
          analogWrite(9,0);
          analogWrite(10,0);
          analogWrite(11,MotorSpeedLevel);
          break;
        case 2:
          //Highside
          PORTD  &= B10010111;
          PORTD  |= B00100000;
          //Lowside
          analogWrite(9,MotorSpeedLevel); 
          analogWrite(10,0);
          analogWrite(11,0);
          break;
       } 
     }

   // Commutation for Regenerative Braking
   /*
   HIGHside transistors are always set to OFF during regen braking.
   AnalogWrite command is used to set the LOWside pins
   (0 = OFF, 255 = ON or brakespeedvalue value that is controlled by the load sensor)
   */

*/
   else{
          //SET HIGHSIDE TO ZERO
            PORTD  &= B10010111;  //keep halls values etc.
            
          switch (HallVal)
         {
          case 3:
            analogWrite(9,BrakeSpeedLevel);
            analogWrite(10,0);
            analogWrite(11,BrakeSpeedLevel);
            break;
          case 1:
            analogWrite(9,0);
            analogWrite(10,BrakeSpeedLevel);
            analogWrite(11,BrakeSpeedLevel);
            break;
          case 5:
            analogWrite(9,BrakeSpeedLevel);
            analogWrite(10,BrakeSpeedLevel);
            analogWrite(11,0);
            break;
          case 4: 
            analogWrite(9,BrakeSpeedLevel);
            analogWrite(10,0);
            analogWrite(11,BrakeSpeedLevel);
            break;
          case 6:
            analogWrite(9,0);
            analogWrite(10,BrakeSpeedLevel);
            analogWrite(11,BrakeSpeedLevel);
          break;
        case 2:
          analogWrite(9,BrakeSpeedLevel);
          analogWrite(10,BrakeSpeedLevel);
          analogWrite(11,0);
          break;
       }
   }  
  }
