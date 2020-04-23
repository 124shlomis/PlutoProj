// in this code motor 1 waiting for *tourqe* (FEED FORWORD) from the serial.
// if there is no load - insert maximal tourqe +- 0.5 [Nm] !!!!!!

#include <mcp_can.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// no joystick deifne
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char len = 0;
unsigned char buf[8]= {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char can_msg[8]= {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/// Value Limits Definition /// need to validate with molex.
const float P_MIN = -15.2*2*PI; 
const float P_MAX = 15.2*2*PI; // AFTER CHECKING. BUT NEED TO CHECK ON THE SETUP MENU (MOLEX) - also, need to check the issue of 12 bits encoder.
const float V_MIN = -45.0; 
const float V_MAX = 45.0; // NEED TO CHECK
const float KP_MIN = 0.0; 
const float KP_MAX = 500.0; // NEED TO CHECK
const float KD_MIN = 0.0;
const float KD_MAX = 5.0; // NEED TO CHECK
const float T_MIN = -18.0;
const float T_MAX = 18.0; // NEED TO CHECK
const float I_MIN = -40.0;
const float I_MAX = 40.0; // NEED TO CHECK

// variables for commands and echo
float p_float = 0;
float v_float = 0;
float i_float = 0;

uint16_t p_int = 0;
uint16_t v_int = 0;
uint16_t i_int = 0;
char idRecived = 0;

// desired angles for drawing a circle:

// --------------- Convertion Functions ------------------------- //

// converts angle to int (for motor input)
unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  
  return ((float)(x - offset) / span * (pow(2,bits)-1)) ;

}

// converts int to angle (for motor output)
float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int bitspan = (pow(2,bits)-1);
  
return  ((float)(x_int * span / bitspan + offset)); //
}
// --------------------------------------------------------------- //

// Initial Command - choose command in physical units

float floatpos = 0; //[rad]
float floatvel = 0; //[rad/s] . "initilized" to  +-45.
float floatkp = 0; //[Nm/rad] . "initilized" to 0 to 500 NM/rad . MAYBE 0-5. need to check im molex.
float floatkd = 0.0; //[rad/s] .   0 to 100 NMs/rad .  need to check im molex.
float floatff = 0.2; //[Nm] . -18 [NM] to +18 [NM] 

  
uint16_t pos = float_to_uint( floatpos , P_MIN , P_MAX , 16 ); // 16 bit 
uint16_t vel = float_to_uint( floatvel , V_MIN , V_MAX , 12 ); // 12 bit 
uint16_t kp = float_to_uint( floatkp , KP_MIN , KP_MAX , 12 );  // 12 bit 
uint16_t kd = float_to_uint( floatkd , KD_MIN , KD_MAX , 12 ); // 12 bit
uint16_t ff = float_to_uint( floatff , T_MIN , T_MAX , 12 );// 12 bit

void getMsg()
{
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
    {
      CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
     idRecived = buf[0];
     p_int = (buf[1]<<8)|buf[2];
     v_int = (buf[3]<<4)|(buf[4]>>4);
     i_int = ((buf[4]&0xF)<<8)|buf[5];

     p_float = uint_to_float(p_int,P_MIN, P_MAX, 16);
     v_float = uint_to_float(v_int,V_MIN, V_MAX, 12);
     i_float = uint_to_float(i_int,I_MIN, I_MAX, 12);
    
      unsigned long canId = CAN.getCanId();
//        SERIAL.print(idRecived,DEC);
//        SERIAL.print(",");
//        SERIAL.print(p_float, DEC);
//        SERIAL.print(",");
        SERIAL.println(v_float, DEC);
//        SERIAL.print(",");
//        SERIAL.println(i_float, DEC);
    }
}

class Motor
{
  public:
    Motor(int id);
    void move(float ff);
    void enterMotorMode();
    void setZeroPosition();
    void exitMotorMode();
    private:
    int id;
    float desired_pos;
};

Motor::Motor(int id)
{
  this->id = id;
}

void Motor::move(float ff)
{
   unsigned int IntFF = float_to_uint(ff, T_MIN, T_MAX, 12);

   can_msg[0] = pos>>8;                                       
   can_msg[1] = pos&0xFF;
   can_msg[2] = vel>>4;
   can_msg[3] = ((vel&0xF)<<4)|(kp>>8);
   can_msg[4] = kp&0xFF;
   can_msg[5] = kd>>4;
   can_msg[6] = ((kd&0xF)<<4)|(IntFF>>8);
   can_msg[7] = IntFF&0xFF;
   CAN.sendMsgBuf(this->id, 0, 8, can_msg);  
//   delay(10);
}

void Motor::enterMotorMode()
{
    stmp[0] = 0xFF;
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFC;
   
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
      SERIAL.println("MSG_FAIL  ENTER MOTOR MODE ID = ");
    }
    else
    {
//      SERIAL.print("MSG_SENT - ENTER MOTOR MODE ID =  ");
//      SERIAL.println(this->id);
    }
    delay(10);
}

void Motor::setZeroPosition()
{
    stmp[0] = 0xFF; //Zero Position Sensor - sets the mechanical position to zero.
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFE;
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
      SERIAL.println(" MSG_FAIL - ZERO ENCODER ID = ");
      SERIAL.println(this->id);
      Serial.println();
      Serial.println();
    }
    else
    {
//      SERIAL.print("MSG_SENT - ZERO ENCODER ID =  ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    delay(10);
}

void Motor::exitMotorMode()
{
    stmp[0] = 0xFF; //Exit Motor Mode 
    stmp[1] = 0xFF;
    stmp[2] = 0xFF;
    stmp[3] = 0xFF;
    stmp[4] = 0xFF;
    stmp[5] = 0xFF;
    stmp[6] = 0xFF;
    stmp[7] = 0xFD;
    if (CAN.sendMsgBuf(this->id, 0, 8, stmp)==CAN_FAIL)
    {
//      SERIAL.println(" MSG_FAIL - EXIT MOTOR MODE ID = ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    else
    {
//      SERIAL.print("MSG_SENT - EXIT MOTOR MODE ID =  ");
//      SERIAL.println(this->id);
//      Serial.println();
//      Serial.println();
    }
    delay(10);
}

// motors defenition:
Motor motor1(1);
Motor motor2(2);

// Arduino start:

void setup()
{
    SERIAL.begin(9600);
    delay(10);
    SERIAL.setTimeout(1000); // not sure for what needed - about reading from serial.
    while (CAN_OK != CAN.begin(CAN_1000KBPS))              // init can bus : baudrate = 1000k
    {
        SERIAL.println("CAN BUS Shield init fail");
        SERIAL.println(" Init CAN BUS Shield again");
        delay(10);
    }
//    SERIAL.println("CAN BUS Shield init ok!");
//    SERIAL.println("Setup over");
//    SERIAL.println();
    
    motor2.setZeroPosition();
    motor2.exitMotorMode();
    motor2.enterMotorMode();
    
    motor1.exitMotorMode();
    motor1.enterMotorMode();
    motor1.setZeroPosition();
    delay(10);
}

float desired_FF = 0.4 ;
float t=0.01; 
void loop()
{
  if (Serial.available() > 0) {
  desired_FF = Serial.parseFloat();
  }
  motor1.move(desired_FF);
  getMsg();
  delay(10);
  t = t + 0.01;
  if (desired_FF == 0){
    delay(100);
    motor1.exitMotorMode();
  }
//  if (t > 5.0) {
//    desired_FF = 0;
//    motor1.move(desired_FF);
//    motor1.exitMotorMode();
//    SERIAL.println(t);
//    exit;
//  }
}

// END FILE
