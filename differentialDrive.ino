/* 
   -- ECE590 - Robot Design --
   Chris Glomb & David Gregory - Homerwork 5
   OpenCM9.04 - Motor Control & USB Interface
   
   Controls 2 Dynamixel motors
*/

 /* Serial device defines for DXL1 bus */ 
#define DXL_BUS_ID 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

/* Dynamixel Motor IDs */
#define RGT_MOTOR 3
#define LFT_MOTOR 2

#define STOP_SPEED  0
#define REVERSE_BIT 0x0400 //10th bit

int16 L_speed = 0;
int16 R_speed = 0;


Dynamixel Dxl(DXL_BUS_ID);



void setup(){
  //You can attach your serialUSB interrupt
  //or, also detach the interrupt by detachInterrupt(void) method
  SerialUSB.attachInterrupt(usbInterrupt);
  
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out; For FUN!!
  
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  Dxl.maxTorque(LFT_MOTOR,1000);
  Dxl.maxTorque(RGT_MOTOR,1000);
  Dxl.wheelMode(LFT_MOTOR); //wheelMode() is to use wheel mode
  Dxl.wheelMode(RGT_MOTOR);
  
  //STOP on RESET
  Dxl.goalSpeed(RGT_MOTOR, STOP_SPEED); 
  Dxl.goalSpeed(LFT_MOTOR, STOP_SPEED);
  
}//End setup()

//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount){
  
  L_speed = (int16)buffer[0] | ((int16)buffer[1])<<8;
  R_speed = (int16)buffer[2] | ((int16)buffer[3])<<8;
  
  SerialUSB.println((int)L_speed);
  
  if(L_speed >= 0){
    Dxl.goalSpeed(LFT_MOTOR, L_speed);
  }
  else{
    Dxl.goalSpeed(LFT_MOTOR, abs(L_speed) | REVERSE_BIT);
  }
  
  if(R_speed >= 0){
    Dxl.goalSpeed(RGT_MOTOR, R_speed | REVERSE_BIT);
  }
  else{
    Dxl.goalSpeed(RGT_MOTOR, abs(R_speed));
  }
}
void loop(){
  //Flash the led
  toggleLED();
  delay(200);
}

