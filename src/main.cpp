#include <Arduino.h>

#include <LoRa.h>                 // Simple LoRa lib

#include "Arduino_APDS9960.h"    // gesture sensor

#include <DFPlayer_Mini_Mp3.h>    // mp3

#include "microLED.h"             // LED strip lib

#include "U8glib.h"               // OLED display

#include <Adafruit_Sensor.h>      // general depends
#include <Adafruit_LSM303_U.h>    // acsels + mag. sensor

//////////////////////////
#include "nRF24L01.h"   // NRF lib files
#include "RF24.h"       //
#include "printf.h"     //
//////////////////////////

////////////////////////////////////
#include <SPI.h>                  // interfaces
#include <Wire.h>                 //
#include <SoftwareSerial.h>       //
#include "OneWire.h"              //
////////////////////////////////////

// NRF_SETTINGS
//////////////////////////////////////////
#define NRF_CHANNEL 103                 // you can choose in range 0-127
#define NRF_DATA_RATE RF24_250KBPS      // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define NRF_PA_LEVEL RF24_PA_MAX        // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm
#define NRF_PIPE  0x1234567899LL        // 
#define NRF_AUTO_ASK false              //
#define NRF_ENABLE_READING_PIPE false   //
#define NRF_READING_PIPE 0x9987654321LL //
//////////////////////////////////////////

// LORA_SETTINGS
//////////////////////////////////
#define LORA_FR long(433E6)     // 433-435 (best)
#define LORA_CRC true           // 
#define LORA_CODING_RATE 8      // 5-11 (mb)
#define LORA_SIG_BW 250E3       // 250E3 - max, look at your lib .h file for more
#define LORA_SP_FACTOR 8        // 8 - best for RA-01 module (at least for mine)
//////////////////////////////////

// baudrates (most of them depends on modules)
/////////////////////////////////////////
#define MP3_BAUDRATE           9600    //
#define COMMUNICATION_BAUDRATE 115200  //
#define RASPBERRY_BAUDRATE     9600    //
/////////////////////////////////////////

// Joystick
////////////////////////
#define JOYSTICK_X A1 //
#define JOYSTICK_Y A2 //
////////////////////////

// Encoder
//////////////////////////
#define ENCODER_CLK 4   //
#define ENCODER_DT  7   //
#define ENCODER_SW  8   //
//////////////////////////

// NRF24L01+
//////////////////////
#define NRF_CE 26   //
#define NRF_CS 27   //
//////////////////////

// LoRa
////////////////////////
#define LORA_NSS  33  //
#define LORA_RST  34  //
#define LORA_DIO0 35  //
////////////////////////

// CC1101
/////////////////////////
#define CC1101_CS   28 //
#define CC1101_GIO0 29 //
/////////////////////////

// mp3 player sotf. uart
////////////////////
#define MP3_TX 30 //
#define MP3_RX 31 //
////////////////////

// LED (5 leds)
////////////////////////
#define LED_PIN 6     //
#define NUM_LEDS 5    // number of leds in strip
#define COLOR_DEBTH 3 // 3 - max
////////////////////////

// buttons
int buttons_pins[4]={48, 49, 50, 51};

// switchers 
///////////////////////////////////////////////////////////////////
const int sw1_pins[10]={36, 37, 38, 39, 40, 41, 42, 43, 44, 45}; //
const int sw2_pins[10]={12, 13, 14, 15, 16, 17, 22, 23, 24, 25}; //
///////////////////////////////////////////////////////////////////

RF24 radio(NRF_CS, NRF_CE);  

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NO_ACK);

////////////////////////////////////////////////////////////////////////////////    
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);         //                     
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);   //                      
////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////                                                    //
LEDdata leds[NUM_LEDS];                         //                                                    //
microLED strip(leds, NUM_LEDS, LED_PIN);        //                                                    //
////////////////////////////////////////////////// 

SoftwareSerial mp3Serial(MP3_TX, MP3_RX); 


////////////////////////////////
struct telemetry              //
{                             //
  uint8_t id=6;               //
  unsigned long time;         //
  uint16_t mode;              //
  uint16_t channel;           //  
  uint16_t joy_x;             //
  uint16_t joy_y;             //
  int16_t enc;                //
  uint8_t buttons;            //
  int16_t acs_x;              //
  int16_t acs_y;              //
  int16_t acs_z;              //  
  int8_t mag_x;               //
  int8_t mag_y;               //
  int8_t mag_z;               //
} radioData;                  //
////////////////////////////////

//for encoder processing:
////////////////////////////////////////////////////
volatile int encCounter;                          //
volatile boolean encflag, encresetFlag;           //
volatile byte enccurState, encprevState;          //
////////////////////////////////////////////////////

bool music_is_on =0;

//////////////////////////////
void sendNrf();             //
void sendLoRa(uint8_t msg); //
void getMode();             //
void getChannel();          //
void getButtons();          //
void getJoyData();          //
void getEncData();          //
void getGesture();          //
void readAcs();             //
void readMag();             //
void dispInfoOled();        //
void dispLed(uint32_t clr); //
void encTick();             //
                            //
void mode1();               //
void mode2();               //
void mode3();               //
void mode4();               //
void mode5();               //
void mode6();               //
void mode7();               //
void mode8();               //
void mode9();               //
void mode10();              //
////////////////////////////// 

void setup() 
{
  pinMode(JOYSTICK_X, INPUT_PULLUP);
  pinMode(JOYSTICK_Y, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(NRF_CE, OUTPUT);
  pinMode(NRF_CS, OUTPUT);
  pinMode(LORA_NSS, OUTPUT);
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  pinMode(CC1101_CS, OUTPUT);
  pinMode(CC1101_GIO0, OUTPUT);
  pinMode(MP3_TX, INPUT);
  pinMode(MP3_RX, OUTPUT);
  for(uint8_t i = 0; i < 4; i++)
  {
    pinMode(buttons_pins[i], INPUT_PULLUP);
  }
  for(uint8_t i = 0; i < 10; i++)
  {
    pinMode(sw1_pins[i], INPUT_PULLUP);
  }
  for(uint8_t i = 0; i < 10; i++)
  {
    pinMode(sw2_pins[i], INPUT_PULLUP);
  }

  attachInterrupt(4, encTick, CHANGE);
  attachInterrupt(7, encTick, CHANGE);

  Serial.begin(COMMUNICATION_BAUDRATE);
  Serial1.begin(115200);
  //////////////////////////////////////////////////////// LoRa config
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);          // 
  if (!LoRa.begin(LORA_FR))                             // 
  {                                                     //
    Serial.println("Starting LoRa failed!");            //
  }                                                     //
  else                                                  //
  {                                                     //
    Serial.println("LoRa started sucsessfully");        //
    LoRa.enableCrc();                                   //
    LoRa.setCodingRate4(LORA_CODING_RATE);              //
    LoRa.setSignalBandwidth(LORA_SIG_BW);               //
    LoRa.setSpreadingFactor(LORA_SP_FACTOR);            //
    LoRa.explicitHeaderMode();                          //
  }                                                     //
  ////////////////////////////////////////////////////////

  ////////////////////////////////////// NRF config (you should check it in #define part)
  printf_begin();                     //
  radio.begin();                      //
  radio.setChannel(NRF_CHANNEL);      //
  radio.setDataRate(NRF_DATA_RATE);   //
  radio.setPALevel(NRF_PA_LEVEL);     //
  radio.openWritingPipe(NRF_PIPE);    //
  radio.setAutoAck(NRF_AUTO_ASK);     //
  radio.printDetails();               //
  //////////////////////////////////////    

  mag.enableAutoRange(true);
  if(!mag.begin()){
    Serial.println("No LSM303 detected.");
  }
  if(!accel.begin()){
    Serial.println("No LSM303 detected.");
  }

  strip.setBrightness(30);
  strip.fill(mHSV(116, 250, 252));
  strip.show();
  mp3Serial.begin(9600);
  mp3_set_serial (mp3Serial);    
  mp3_set_volume (30);
  mp3_play(1);
  delay(1400);
 // mp3_pause();

}

void loop() 
{
  getMode();
  getChannel();
  getButtons();
  getJoyData();
  getEncData();
  readAcs();
  readMag();
  strip.fill(mHSV(116, 250, 252));
  strip.show();
  /*
      Data transfer to raspberry pi pico
      1: mode
      2: channel
      3:joy x
      4:joy y
      5:enc
      6:buttons
      7:acs x
      8:acs y
      9:acs z
      a:mag x
      b:mag y
      c:mag z




  */
  Serial1.println("1"+String(radioData.mode));
  Serial1.println("2"+String(radioData.channel));

  Serial1.println("3"+String(radioData.joy_x));
  Serial1.println("4"+String(radioData.joy_y));
  
  Serial1.println("5"+String(radioData.enc));
  
  Serial1.println("6"+String(radioData.buttons));

  Serial1.println("7"+String(radioData.acs_z));
  Serial1.println("8"+String(radioData.acs_x));
  Serial1.println("9"+String(radioData.acs_y));

  Serial1.println("a"+String(radioData.mag_x));
  Serial1.println("b"+String(radioData.mag_y));
  Serial1.println("c"+String(radioData.mag_z));
  

  
  u8g.firstPage();
  do
  {
    dispInfoOled();
  } while (u8g.nextPage());
  switch (radioData.mode)
  {
  case 0:
    mode1();
    if(music_is_on)
      mp3_pause();
    break;
  case 1:
    mode2();
    break;
  case 2:
    mode3();
    break;
  case 3:
    mode4();
    break;
  case 4: 
    mode5();
    break;
  case 5:
    mode6();
    break;
  case 6:
    mode7();
    break;
  case 7:
    mode8();
    break;
  case 8:
    mode9();
    break;
  case 9: 
    mode10();
    if(!music_is_on)
    {
        mp3_play(2);
        music_is_on=1;
    }
    break;
  }

}

void sendNrf()             
{
  radio.write(&radioData, sizeof(radioData));
}

void sendLoRa(uint8_t msg) 
{
  LoRa.beginPacket();
  switch (msg)
  {
  case 0:
    break;
  case 1:
    LoRa.print(radioData.acs_x);
    LoRa.print(" ");
    LoRa.print(radioData.acs_y);
    LoRa.print(" ");
    LoRa.print(radioData.acs_z);
    break;
  case 2:
    LoRa.print(radioData.mag_x);
    LoRa.print(" ");
    LoRa.print(radioData.mag_y);
    LoRa.print(" ");
    LoRa.print(radioData.mag_z);
    break;
  case 3:
    LoRa.print(radioData.enc);
    LoRa.print(" ");
    LoRa.print(radioData.joy_x);
    LoRa.print(" ");
    LoRa.print(radioData.joy_y);
    break;
  case 4:
    LoRa.print(radioData.mode);
    LoRa.print(" ");
    LoRa.print(radioData.channel);
    LoRa.print(" ");
    LoRa.print(radioData.buttons);
    break;
  }
  LoRa.endPacket(1);
}

void getMode()          
{
  bitWrite(radioData.mode, 0, (!digitalRead(sw1_pins[0])));
  bitWrite(radioData.mode, 1, (!digitalRead(sw1_pins[1])));
  bitWrite(radioData.mode, 2, (!digitalRead(sw1_pins[2])));
  bitWrite(radioData.mode, 3, (!digitalRead(sw1_pins[3])));
  bitWrite(radioData.mode, 4, (!digitalRead(sw1_pins[4])));
  bitWrite(radioData.mode, 5, (!digitalRead(sw1_pins[5])));
  bitWrite(radioData.mode, 6, (!digitalRead(sw1_pins[6])));
  bitWrite(radioData.mode, 7, (!digitalRead(sw1_pins[7])));
  bitWrite(radioData.mode, 8, (!digitalRead(sw1_pins[8])));
  bitWrite(radioData.mode, 9, (!digitalRead(sw1_pins[9])));
  
}

void getChannel()       
{
  bitWrite(radioData.channel, 0, (!digitalRead(sw2_pins[0])));
  bitWrite(radioData.channel, 1, (!digitalRead(sw2_pins[1])));
  bitWrite(radioData.channel, 2, (!digitalRead(sw2_pins[2])));
  bitWrite(radioData.channel, 3, (!digitalRead(sw2_pins[3])));
  bitWrite(radioData.channel, 4, (!digitalRead(sw2_pins[4])));
  bitWrite(radioData.channel, 5, (!digitalRead(sw2_pins[5])));
  bitWrite(radioData.channel, 6, (!digitalRead(sw2_pins[6])));
  bitWrite(radioData.channel, 7, (!digitalRead(sw2_pins[7])));
  bitWrite(radioData.channel, 8, (!digitalRead(sw2_pins[8])));
  bitWrite(radioData.channel, 9, (!digitalRead(sw2_pins[9])));
  
}

void getButtons()      
{
  radioData.buttons = 1000*!digitalRead(buttons_pins[0])+
                      100* !digitalRead(buttons_pins[1])+
                      10 * !digitalRead(buttons_pins[2])+
                      1 * !digitalRead( buttons_pins[3]);
}

void getJoyData()          
{
  radioData.joy_x = analogRead(JOYSTICK_X);
  radioData.joy_y = analogRead(JOYSTICK_Y);
}

void getEncData()      
{
  radioData.enc = encCounter;
}

void encTick() 
{
  enccurState = digitalRead(ENCODER_CLK) | digitalRead(ENCODER_DT) << 1;  
  if (encresetFlag && enccurState == 0b11) 
  {
    if (encprevState == 0b10) 
      encCounter++;
    if (encprevState == 0b01) 
      encCounter--;
    encresetFlag = 0;
    encflag = true;
  }
  if (enccurState == 0b00) encresetFlag = 1;
  encprevState = enccurState;
}

void getGesture()       
{
  // to do (bagged reading from sensor because of changed I2C adress)
}

void readAcs()          
{
  sensors_event_t event;
  accel.getEvent(&event);
  radioData.acs_x = event.acceleration.x;
  radioData.acs_y = event.acceleration.y;
  radioData.acs_z = event.acceleration.z;
}

void readMag()             
{
  sensors_event_t event;
  mag.getEvent(&event);
  radioData.mag_x = event.magnetic.x;
  radioData.mag_y = event.magnetic.y;
  radioData.mag_z = event.magnetic.z;
}

void dispInfoOled()        
{
  Wire.beginTransmission(0x08); 
  Wire.write(radioData.mode);
  Wire.write(radioData.channel);
  Wire.write(radioData.joy_x);
  Wire.write(radioData.joy_y);
  Wire.write(radioData.enc);
  Wire.write(radioData.buttons);
  Wire.write(radioData.acs_x);
  Wire.write(radioData.acs_y);
  Wire.write(radioData.acs_z);
  Wire.write(radioData.mag_x);
  Wire.write(radioData.mag_y);
  Wire.write(radioData.mag_z);
  Wire.endTransmission();
  u8g.setFont(u8g_font_unifont);
  u8g.setPrintPos(2, 10); 
  u8g.print(radioData.mode);
  u8g.print(" "); 
  u8g.print(radioData.channel);
  u8g.setPrintPos(2, 21); 
  u8g.print(radioData.buttons);
  u8g.print(" ");
  u8g.print(radioData.enc);
  u8g.setPrintPos(2, 33); 
  u8g.print(radioData.joy_x);
  u8g.print(" ");
  u8g.print(radioData.joy_y);
  u8g.setPrintPos(2, 44); 
  u8g.print(radioData.acs_x);
  u8g.print(" ");
  u8g.print(radioData.acs_y);
  u8g.print(" ");
  u8g.print(radioData.acs_z);
  u8g.setPrintPos(2, 55); 
  u8g.print(radioData.mag_x);
  u8g.print(" ");
  u8g.print(radioData.mag_y);
  u8g.print(" ");
  u8g.print(radioData.mag_z);
  //u8g.nextPage();
  
}


void mode1()  // simple radio transmitting
{
  sendNrf();
}

void mode2()
{
  sendLoRa(radioData.channel); 
}

void mode3()
{

}

void mode4()
{

}

void mode5()
{

}

void mode6()
{

}

void mode7()
{

}

void mode8()
{

}

void mode9()
{

}

void mode10()
{

}
