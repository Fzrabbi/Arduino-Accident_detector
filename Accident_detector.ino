#include <Wire.h> /declaration of library
#include <LiquidCrystal_I2C.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <DFRobot_sim808.h>
#include <SoftwareSerial.h>

#define MESSAGE_LENGTH 160
char message[MESSAGE_LENGTH];
int messageIndex = 0;
char MESSAGE[300];
char lat[12];//latitute 
char lon[12];//londitute
char wspeed[12];
const int buzzer = 9;

char phone[16];
char datetime[24];

#define PIN_TX    10
#define PIN_RX    11
SoftwareSerial mySerial(PIN_TX,PIN_RX);
DFRobot_SIM808 sim808(&mySerial);//Connect RX,TX,PWR,


MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

#define BACKLIGHT_PIN     13

#define x A1
#define y A2
#define z A3

int xsample=0;
int ysample=0;
int zsample=0;

#define samples 10

#define minVal -5000 //treshold value for detecting accident
#define MaxVal 5000

int i=0,k=0;

//LiquidCrystal_I2C lcd(0x38);  // Set the LCD I2C address

LiquidCrystal_I2C lcd(0x27 ,2,1,0,4,5,6,7,3, POSITIVE);  // Set the LCD I2C address


// Creat a set of new characters

void setup()
{
  mySerial.begin(9600);
  Serial.begin(9600);
  // Switch on the backlight

  pinMode(buzzer, OUTPUT);
  pinMode ( BACKLIGHT_PIN, OUTPUT );
  digitalWrite ( BACKLIGHT_PIN, HIGH );
  
  lcd.begin(16,2);               // initialize the lcd 

  lcd.home ();                   // go home
  lcd.print("Accident Alert  ");
  lcd.setCursor(0,1);
  lcd.print("     System     ");
  delay(2000);
  lcd.clear();
  lcd.print("Initializing");
  delay(1500);
  
  while(!sim808.init())
  {
      lcd.setCursor(0,0);
      Serial.print("Sim init error\r\n");
      lcd.print("Sim init error");
      delay(1000);
      
      
      
  }
  lcd.clear();
  lcd.print("Sim808 working fine");
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Please Wait...");
    mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
   }
  delay(1000);

  lcd.clear();
  lcd.print("Callibrating ");
  lcd.setCursor(1,0);
  lcd.print("Taking samples"); 
  delay(1000);
  for(int i=0;i<samples;i++)////taking 10 samples and avarage them to mesure the mean accelaration 
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    xsample+=gx;
    ysample+=gy;
    zsample+=gz;
  }
  xsample/=samples;
  ysample/=samples;
  zsample/=samples;
  
  Serial.print("xsample");Serial.println(xsample);
  Serial.print("ysample");Serial.println(ysample);
  Serial.print("zsample");Serial.println(zsample);
  delay(1000);
  
  lcd.clear();
  lcd.print("Checking GPS");//checking the GPS device is connected or not
  delay(1500);
  lcd.setCursor(1,0);
  if( sim808.attachGPS()){
      Serial.println("GPS success");
      lcd.print("GPS success");
      delay(2500);
  }
  else{ 
      lcd.print("GPS power fail");
      Serial.println("Open the GPS failure");
      delay(2500);
  }
}

void loop()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //caling the mpu6050 values
  //Serial.print("AcX = "); Serial.print(AcX);
  //Serial.print(" | AcY = "); Serial.print(AcY);
  //Serial.print(" | AcZ = "); Serial.print(AcZ);
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); Serial.print(GyX);
  //Serial.print(" | GyY = "); Serial.print(GyY);
  //Serial.print(" | GyZ = "); Serial.println(GyZ);
    int value1=gx;
    int value2=gy;
    int value3=gz;
    //Serial.println("x=");Serial.print(value1);
    //Serial.println("y=");Serial.print(value2);
    //Serial.println("z=");Serial.print(value3);
    int xValue=xsample-value1;
    int yValue=ysample-value2;
    int zValue=zsample-value3;
    
    Serial.print("x=");
    Serial.println(xValue);
    Serial.print("y=");
    Serial.println(yValue);
    Serial.print("z=");
    Serial.println(zValue);
    lcd.print("Stable");
    delay(100);
    lcd.setCursor(0,1);
   if(xValue < minVal || xValue > MaxVal  || yValue < minVal || yValue > MaxVal  || zValue < minVal || zValue > MaxVal) //measuring the current value with the mean values
    {
      //get_gps();
      //show_coordinate();
      lcd.clear();
      lcd.print("Accident detected ");
      tone(buzzer, 1000); // Send 1KHz sound signal...
      delay(1000); 
      delay(2000);
      delay(3000);// ...for 1 sec
      noTone(buzzer);     // Stop sound...
      delay(1000);        // ...for 1sec
      
      lcd.setCursor(1,1);
      while(!sim808.getGPS())
      {
         
      }
      lcd.print("Sending SMS ");
      delay(1000);
      Serial.print(sim808.GPSdata.year);
      Serial.print("/");
      Serial.print(sim808.GPSdata.month);
      Serial.print("/");
      Serial.print(sim808.GPSdata.day);
      Serial.print(" ");
      Serial.print(sim808.GPSdata.hour);
      Serial.print(":");
      Serial.print(sim808.GPSdata.minute);
      Serial.print(":");
      Serial.print(sim808.GPSdata.second);
      Serial.print(":");
      Serial.println(sim808.GPSdata.centisecond);
      Serial.print("latitude :");
      Serial.println(sim808.GPSdata.lat);
      Serial.print("longitude :");
      Serial.println(sim808.GPSdata.lon);
      Serial.print("speed_kph :");
      Serial.println(sim808.GPSdata.speed_kph);
      Serial.print("heading :");
      Serial.println(sim808.GPSdata.heading);
      Serial.println();
      float la = sim808.GPSdata.lat; //taking the gps location
      float lo = sim808.GPSdata.lon;
      float ws = sim808.GPSdata.speed_kph;
  
      dtostrf(la, 6, 2, lat); //put float value of la into char array of lat. 6 = number of digits before decimal sign. 2 = number of digits after the decimal sign.
      dtostrf(lo, 6, 2, lon); //put float value of lo into char array of lon
      dtostrf(ws, 6, 2, wspeed);  //put float value of ws into char array of wspeed
      //fetching it into a text message
      sprintf(MESSAGE, "Latitude : %s\nLongitude : %s\nWind Speed : %s kph\nMy Module Is Working. Mewan Indula Pathirage. Try With This Link.\nhttp://www.latlong.net/Show-Latitude-Longitude.html\nhttp://maps.google.com/maps?q=%s,%s\n", lat, lon, wspeed, lat, lon);

      sim808.sendSMS("+8801701081034",MESSAGE); //sending the message to certain phone number
  
      //************* Turn off the GPS power ************
      sim808.detachGPS();
      Serial.println("Sending SMS");
      //Send();
      Serial.println("SMS Sent");
      lcd.clear();
      lcd.print("massage sent to");
      lcd.setCursor(1,1);
      lcd.print("01701081034");
      delay(3000);
      lcd.clear();
      lcd.print("stable");
      
    }       
   lcd.clear();
   
}
