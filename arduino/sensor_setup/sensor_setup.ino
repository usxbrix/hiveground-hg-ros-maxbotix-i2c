/* Code for Arduino Leonardo
Assumes the sensor is using the default address
Sensor Connections:
Pin 7 to GND 
Pin 6 to 5V
Pin 5 to SCL
Pin 4 to SDA
Requires pull‑ups for SCL and SDA connected to 5V to work reliably
*/
#include <Wire.h>
#include "ascii.h"

//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
#define SensorAddressDefault byte(0x70)

//The Sensor ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address
#define ChangeAddressCommand1 byte(0xAA)
#define ChangeAddressCommand2 byte(0xA5)

void setup() {
  Serial.begin(9600);//Open serial connection at 9600 baud
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Wire.begin();//Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
}


char key = 0;
byte temp = 0;
byte sensor_address = SensorAddressDefault;
byte sensor_new_address = sensor_address;
byte start_address, end_address;
word range;

void showMenu()
{
  Serial.println("I2CXL Setup");  
  Serial.println(" 1. Test sensor");
  Serial.println(" 2. Change address");
  Serial.println(" 3. Test multiple sensors");
  Serial.print("> select [1-3]:");
}

char readCharBlocking()
{
  while(!Serial.available()) ;
  return Serial.read();
}

int line_length = 0;
char incoming_line[64];
void readLine()
{
  line_length=0;
  do
  {
    incoming_line[line_length] = readCharBlocking();
    line_length++;
  } while (incoming_line[line_length-1] != ASCII_ENTER && (line_length < 64));
  incoming_line[line_length] = 0;
}

void loop() {
  showMenu();
  readLine();
  Serial.println(incoming_line[0]);
  switch(incoming_line[0])
  {
    case '1': 
      {
        Serial.print("Enter address:");
        readLine();
        sscanf(incoming_line, "%x", &sensor_address);
        Serial.print("0x");Serial.println(sensor_address, HEX);
        Serial.println("Enter 'q' to stop");
        while(1)
        {
          takeRangeReading(sensor_address);                                       //Tell the sensor to perform a ranging cycle          
          delay(100);                                                    //Wait for the sensor to finish  
          range = requestRange(sensor_address);                           //Get the range from the sensor
          Serial.print("Range:");Serial.println(range);          //Print to the user
          readLine();
          if(incoming_line[0] == 'q')
            break;
        }
      }    
      break;    
    case '2': 
      Serial.print("Enter old address:"); 
      readLine();
      sscanf(incoming_line, "%x", &sensor_address);
      Serial.print("0x");Serial.println(sensor_address, HEX);

      Serial.print("Enter new address:"); 
      readLine();
      sscanf(incoming_line, "%x", &sensor_new_address);
      Serial.print("0x");Serial.println(sensor_new_address, HEX);

      changeAddress(sensor_address, sensor_new_address, true);
      delay(1000);
      Serial.println("Done!");
      break;
    case '3':
      Serial.print("Enter start address:"); 
      readLine();
      sscanf(incoming_line, "%x", &start_address);
      Serial.print("0x");Serial.println(start_address, HEX);

      Serial.print("Enter end address:"); 
      readLine();
      sscanf(incoming_line, "%x", &end_address);
      Serial.print("0x");Serial.println(sensor_new_address, HEX);
      
      while(1)
      {
        for(byte i = start_address; i <= end_address; i++)
        {
          takeRangeReading(i);
        } 
        delay(100);      
        for(byte i = start_address; i <= end_address; i++)
        {
          range = requestRange(i);           //Get the range from the sensor
          Serial.print(range);Serial.print(" ");         //Print to the user     
        } 
        Serial.println("");
        if(Serial.read() == 'q')
          break;      
      }
    default: 
      break;
  }

 
/* 
  if(val == 0 && !addressChanged)
  {
    Serial.print("Change address from:");
    Serial.print(SensorAddressDefault);
    Serial.print(" to:");
    Serial.println(SensorAddress);
    changeAddress(SensorAddressDefault, SensorAddress, true);
    delay(10);
    while(true) { }
  }

  

  if(valRead == 0)
  {    
    takeRangeReading(SensorAddress0);                                       //Tell the sensor to perform a ranging cycle
    takeRangeReading(SensorAddress1);                                       //Tell the sensor to perform a ranging cycle
    delay(100);                                                    //Wait for the sensor to finish  
    word range0 = requestRange(SensorAddress0);                           //Get the range from the sensor
    word range1 = requestRange(SensorAddress1);                           //Get the range from the sensor
    Serial.print("0x60:");Serial.print(range0);          //Print to the user
    Serial.print("0x61:");Serial.println(range1);          //Print to the user
  }
*/  
}

//Commands the sensor to take a range reading
void takeRangeReading(byte address){
  Wire.beginTransmission(address);             //Start addressing 
  Wire.write(RangeCommand);                             //send range command 
  Wire.endTransmission();                                  //Stop and do something else now
}    

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication. 
word requestRange(byte address){ 
  Wire.requestFrom(address, byte(2));
  if(Wire.available() >= 2){                            //Sensor responded with the two bytes 
    byte HighByte = Wire.read();                        //Read the high byte back 
    byte LowByte = Wire.read();                        //Read the low byte back 
    word range = word(HighByte, LowByte);         //Make a 16-bit word out of the two bytes for the range 
    return range;   
  }
  else 
  { 
    return word(0);                                             //Else nothing was received, return 0 
  }
}

/* Commands a sensor at oldAddress to change its address to newAddress 
oldAddress must be the 7-bit form of the address that is used by Wire 
7BitHuh determines whether newAddress is given as the new 7 bit version or the 8 bit version of the address 
\ If true, if is the 7 bit version, if false, it is the 8 bit version 
*/
void changeAddress(byte oldAddress, byte newAddress, boolean SevenBitHuh){ 
  Wire.beginTransmission(oldAddress);                 //Begin addressing
  Wire.write(ChangeAddressCommand1);              //Send first change address command
  Wire.write(ChangeAddressCommand2);              //Send second change address command 
 
  byte temp;
  if(SevenBitHuh){ temp = newAddress << 1; }     //The new address must be written to the sensor
  else     { temp = newAddress;         }               //in the 8bit form, so this handles automatic shifting
  Wire.write(temp);                                          //Send the new address to change to 
  Wire.endTransmission();
}


