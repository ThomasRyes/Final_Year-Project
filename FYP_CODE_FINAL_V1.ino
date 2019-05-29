
#include <SoftwareSerial.h>
#include <Wire.h>
#include <math.h>

#define Rx 4 //Digital pin 4 on arduino
#define Tx 5 //Digital pin 5 on arduino

SoftwareSerial BTSerial(Rx, Tx); //For Bluetooth module

#define AccelSens_2gs           16384.0
#define AccelSens_4gs           8192.0
#define AccelSens_8gs           4096.0
#define AccelSens_16gs          2048.0

#define GyroSens_250_dps         131.0
#define GyroSens_500_dps         65.5
#define GyroSens_1000_dps        32.8
#define GyroSens_2000_dps        16.4

#define AFS_SEL_0                B00000000
#define AFS_SEL_1                B00001000
#define AFS_SEL_2                B00010000
#define AFS_SEL_3                B00011000

#define GFS_SEL_0                B00000000
#define GFS_SEL_1                B00001000
#define GFS_SEL_2                B00010000
#define GFS_SEL_3                B00011000

#define Gyro_Output_Rate         8000
#define Accel_Output_Rate        1000

#define SMPLRT_DIV_Reg           0x19
#define Gyro_Accel_LPF           0x1A

#define SMPLRT_DIV_Value      B10010111 //Setting sample rate divide value set to 152 (decimal)

//=======================================================================================================================//

//VARIABLE DECLARATIONS


//MPU Variables
long Raw_AX = 0 , Raw_AY = 0, Raw_AZ = 0; //varaibles to store the raw accel values
float AX, AY, AZ;                         //variables to store the accel values in Gs

long Raw_GX = 0, Raw_GY = 0, Raw_GZ = 0; //varaibles to store the raw gyro values
float GX, GY, GZ;                        //variables to store the gyro values in degrees/second

float angle_AX, angle_AY, angle_AZ;
float angle_GX , angle_GY, angle_GZ;

//float rads_to_degrees = 180/3.14159; 
float rads_to_degrees = 180/PI;

float sampleRate; 
float dT;

float PITCH, ROLL, YAW;

const int interval = 19; //19ms
unsigned long previousTime = 0 ;
unsigned long currentTime = 0;
unsigned long elapsedTime = 0;

//=======================================================================================================================//

//MAIN SETUP AND LOOP FUNCTIONS

void setup()
{
  BTSerial.begin(9600);
  Serial.begin(9600);
  Wire.begin();

  pinMode(Rx, INPUT);
  pinMode(Tx, OUTPUT);
  
  initialise_MPU();

  
 
}

void loop()
{
  get_sampleRate();
  get_Raw_AccelData();
  get_Raw_GyroData();
  convertData();
  get_accel_gyro_angles();
  sensor_Fusion();
  printData();
 
}

//=======================================================================================================================//

//FUNCTION DEFINITIONS FOR SENSOR

void initialise_MPU()
{
  Wire.beginTransmission(B1101000); //This is the slave address of the MPU - datasheet
  Wire.write(0x6B); //Accessing the register for power management
  Wire.write(B00000000); //Set SLEEP register to 0. - we don't want the MPU to stop midway
  Wire.endTransmission();  

  Wire.beginTransmission(B1101000); //slave address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration 
  Wire.write(AFS_SEL_3); //Setting the accel to +/- 16g
  Wire.endTransmission(); 
  
  Wire.beginTransmission(B1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration 
  Wire.write(GFS_SEL_3 ); //Setting the gyro to full scale +/- 2000dps
  Wire.endTransmission(); 

  Wire.beginTransmission(B1101000); //I2C address of the MPU
  Wire.write(0x19); //Accessing the register 19 for sample rate divide register
  Wire.write(SMPLRT_DIV_Value ); //Setting value to 152 in decimal
  Wire.endTransmission();
  
  
}

void get_Raw_AccelData ()
{
  Wire.beginTransmission(B1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(B1101000,6); //Request access to accel registers 0x3B - 0x40 (in the datasheet)
  
  while(Wire.available() < 6);
  
  Raw_AX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  Raw_AY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  Raw_AZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  
  
}

void get_Raw_GyroData() 
{
  
  Wire.beginTransmission(B1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  
  Wire.requestFrom(B1101000,6); //Request access to gyro resigters 0x43 - 0x48 (in the datasheet)
  
  while(Wire.available() < 6);
  
  Raw_GX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyroX
  Raw_GY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyroY
  Raw_GZ = Wire.read()<<8|Wire.read(); //Store last two bytes into gyroZ

}

void convertData()
{
  
   //16384.0 = LSB/g ---> for +-2g sensitivity
  //8192.0 = LSB/g ---> for +-4g sensitivity
  //4096.0 = LSB/g ---> for +-8g sensitivity
  //2048.0 = LSB/g ---> for +-16g sensitivity

  
  AX = Raw_AX  / AccelSens_16gs; //to get linear acceleration in Gs
  AY = Raw_AY / AccelSens_16gs; 
  AZ = Raw_AZ / AccelSens_16gs;

  

  //131.0 = LSB per Deg/s --> for +-250dps sensitivity
  //65.5 = LSB per Deg/s --> for +-500dps sensitivity
  //32.8 = LSB per Deg/s --> for +-1000dps sensitivity
  //16.4 = LSB per Deg/s --> for +-2000dps sensitivity

 
  GX = Raw_GX / GyroSens_2000_dps; //to get angular velocity in degrees per second
  GY = Raw_GY / GyroSens_2000_dps;
  GZ = Raw_GZ / GyroSens_2000_dps;
 
 
  
}

void get_sampleRate()
{
   sampleRate = Gyro_Output_Rate / (1 + SMPLRT_DIV_Value );
  //sample rate is 52 hertz 

  //dT = 1/sampleRate;
  //dT = 19ms

}

void get_accel_gyro_angles()
{


  
  //atan2 function calculates inverse tan from all four quadrants 
  angle_AX = atan2(  AX , sqrt (AY*AY+ AZ*AZ)  ) * rads_to_degrees;
  angle_AY = atan2( AY , sqrt (AX*AX + AZ*AZ)  ) * rads_to_degrees;
  angle_AZ = 0; //Accelerometer does not give AZ readings





  currentTime = millis();
  previousTime = currentTime;
  elapsedTime = (currentTime - previousTime)/interval;
  
  //Gyro Angles
  angle_GX += GX*(elapsedTime);
  angle_GY += GY*(elapsedTime);
  angle_GZ += GZ*(elapsedTime);

  elapsedTime = currentTime;
 
  
}


void sensor_Fusion()
{
  //Applying the complementary filter
  ROLL = ((0.981) * angle_GX + (1.0 - 0.981)* angle_AX ) * rads_to_degrees;  
  PITCH = ((0.981) * angle_GY + (1.0 - 0.981)* angle_AY) * rads_to_degrees; 
  YAW =  angle_GZ;
 
}






void printData()
{


    Serial.println(PITCH);
  
        if(BTSerial.available() > 0)//if data available for bluetooth is greater than 0
                {
                     
                      BTSerial.println(PITCH);
              
                } 
              delay(50); 
       
        }
  
  
}












