#include <Wire.h>
#include <PixyI2C.h>
#include <LiquidCrystal_I2C.h>

//LCD
#define I2C_ADDR    0x3F
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7

//////////////////////////////
#define X_CENTER    160L  
#define Y_CENTER    100L 
#define RCS_MIN_POS     0L 
#define RCS_MAX_POS     1000L 
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

#define BRAKE 0
#define Forward  1
#define Backward 2

#define MOTOR_1 0 
#define MOTOR_2 1

const byte PWM1 = 5;  // PWM control (speed) for motor A
const byte PWM2 = 6; // PWM control (speed) for motor B
//const byte DIRA = 12; // Direction control for motor A
//const byte DIRB = 13; // Direction control for motor B

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9


#define CURRENT_SEN_1 A2
#define CURRENT_SEN_2 A3

#define EN_PIN_1 A0
#define EN_PIN_2 A1

class ServoLoop
{
public:
    ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
  
    void update(int32_t error);
  
    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_proportionalGain;
    int32_t m_derivativeGain;
};
  
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
    m_pos = RCS_CENTER_POS;
    m_proportionalGain = proportionalGain;
    m_derivativeGain = derivativeGain;
 
    m_prevError = 0x80000000L;
}
  
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
    long int velocity;
    char buf[32];
    if (m_prevError!=0x80000000)
    {
        velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
  
        m_pos += velocity;
        if (m_pos>RCS_MAX_POS) 
        {
            m_pos = RCS_MAX_POS; 
        }
        else if (m_pos<RCS_MIN_POS)
        {
            m_pos = RCS_MIN_POS;
        }
    }
    m_prevError = error;
}

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(200, 200);
PixyI2C pixy(0x55);
LiquidCrystal_I2C  lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

void setup() {
   Serial.begin(9600);
   Serial2.begin(9600);
   Serial.print("Starting...\n");
   pinMode(48,OUTPUT);
   pinMode(49,OUTPUT);
 //  pixy.init();
   setupArdumoto(); //ฟังก์ชั่นกำหนด Pin และโหมดการทำงานของ Motor
   // initialize the LCD
  // lcd.begin(16,2);
   // Turn on the blacklight and print a message.
  // lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  // lcd.backlight();
  // lcd.setCursor(0, 0);
}

uint32_t lastBlockTime = 0;
int tracking = 0;
short leftSpeed = 0;
short rightSpeed = 0;
int mode = 0;

void loop() {
  uint16_t blocks;
    /*blocks = pixy.getBlocks();
    // If we have blocks in sight, track and follow them
    for (int n = 0; n < blocks; n++){
    if(pixy.blocks[n].signature == 668) {tracking = 1; break; } else tracking = 0;  }
    switch(tracking){
      case 0 : {digitalWrite(49,HIGH);digitalWrite(48,LOW);} break;
      case 1 : {digitalWrite(48,HIGH);digitalWrite(49,LOW);} break;
    }*/
    getSerialMobile();
  /*  Serial.print("left: ");
    Serial.println(leftSpeed);
    Serial.print("right: ");
    Serial.println(rightSpeed);*/
    // And set the motor speeds
        if(leftSpeed > 0)
        {
          driveArdumoto(MOTOR_2, Forward, leftSpeed);
        }
        if(rightSpeed > 0)
        {
          driveArdumoto(MOTOR_1, Forward, rightSpeed);
        }
        if(leftSpeed < 0)
        {
          driveArdumoto(MOTOR_2, Backward, leftSpeed);
        }
        if(rightSpeed < 0)
        {
          driveArdumoto(MOTOR_1, Backward, rightSpeed);
        }
        if(leftSpeed==0&&rightSpeed==0)
        {
          stopArdumoto(MOTOR_1);
          stopArdumoto(MOTOR_2);
        }
   /* if (blocks && tracking == 1)
    {
        lcd.clear();
        int trackedBlock = TrackBlock(blocks);
        FollowBlock(trackedBlock);
        lastBlockTime = millis();
    }
    else if (millis() - lastBlockTime > 100)
    {
        
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Not Detected");
        tracking = 0;
        stopArdumoto(MOTOR_1);
        stopArdumoto(MOTOR_2);
    } */
}

void getSerialMobile()
{
  String str,str1;
  int i,j,amount,index_start;
  if(Serial2.available()>0)
  {
    str = "";
    str = Serial2.readStringUntil('\0');
    
    for(i=0;i<str.length();i++)
    {
      if(str[i]=='L'&&str[i+1]=='=')
      {
        index_start = i+2;
        amount = 0;
        for(j=i+2;j<str.length();j++)  // Search index stop
        {
          if(str[j]!=',') amount++; else break;
        }
        str1 = "";
        for(i=index_start;i<(index_start+amount);i++)
        str1 += str[i];
        leftSpeed = str1.toInt();
      }

      if(str[i]=='R'&&str[i+1]=='=')
      {
        index_start = i+2;
        amount = 0;
        for(j=i+2;j<str.length();j++)  // Search index stop
        {
          if(str[j]!=',') amount++; else break;
        }
        str1 = "";
        for(i=index_start;i<(index_start+amount);i++)
        str1 += str[i];
        rightSpeed = str1.toInt();
      }
      if(str[i]=='M'&&str[i+1]=='o'&&str[i+2]=='d'&&str[i+3]=='e'&&str[i+1]=='=')
      {
        mode = atoi(str[5]);
      }
    }
  }
}
int oldX, oldY, oldSignature;
int TrackBlock(int blockCount)
{
    int trackedBlock = 0;
    long maxSize = 0;
    Serial.print("blocks =");
    Serial.println(blockCount);
    for (int n = 0; n < blockCount; n++)
    {
        if ((oldSignature == 0) || (pixy.blocks[n].signature == oldSignature))
        {
            long newSize = pixy.blocks[n].height * pixy.blocks[n].width;
            if (newSize > maxSize)
            {
                trackedBlock = n;
                maxSize = newSize;
            }
        }
    }

    int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
    int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    oldX = pixy.blocks[trackedBlock].x;
    oldY = pixy.blocks[trackedBlock].y;
    oldSignature = pixy.blocks[trackedBlock].signature;
    return trackedBlock;
}

int32_t size = 500;
void FollowBlock(int trackedBlock)
{
    int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?

    Serial.println(followError);
    lcd.setCursor(2, 0);
    lcd.print("FW: ");
    lcd.print(followError);
    // Size is the area of the object.
    // We keep a running average of the last 8.
    size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
    size -= size >> 3;
  
    // Forward speed decreases as we approach the object (size is larger)
    int forwardSpeed = constrain(500 - (size/256), -250, 250);
    // Steering differential is proportional to the error times the forward speed
    int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
    // Adjust the left and right speeds by the steering differential.
    leftSpeed = constrain(forwardSpeed + differential, -20, 20);
    rightSpeed = constrain(forwardSpeed - differential, -20, 20);
    Serial.print("left: ");
    Serial.println(leftSpeed);
    Serial.print("right: ");
    Serial.println(rightSpeed);

    // And set the motor speeds
        if(leftSpeed > 0)
        {
          driveArdumoto(MOTOR_2, Forward, leftSpeed);
        }
        if(rightSpeed > 0)
        {
          driveArdumoto(MOTOR_1, Forward, rightSpeed);
        }
        if(leftSpeed < 0)
        {
          driveArdumoto(MOTOR_2, Backward, leftSpeed);
        }
        if(rightSpeed < 0)
        {
          driveArdumoto(MOTOR_1, Backward, rightSpeed);
        }
}

void driveArdumoto(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if(motor == MOTOR_1)
  {
    if(direct == Forward)
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == Backward)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM1, pwm); 
  }
  else if(motor == MOTOR_2)
  {
    if(direct == Forward)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == Backward)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM2, pwm);
  }
}

void stopArdumoto(byte motor)
{
  driveArdumoto(motor,BRAKE,0);
}

void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  
  pinMode(CURRENT_SEN_1, OUTPUT);
  pinMode(CURRENT_SEN_2, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
 
  // Initialize all pins as low:
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
}
