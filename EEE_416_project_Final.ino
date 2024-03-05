#include <SoftwareSerial.h>
SoftwareSerial sim800l(10, 11);
#include <SoftwareSerial.h>
#include <TinyGPS.h>
SoftwareSerial mySerial(7, 8); //for gps
TinyGPS gps;


#define ignition_switch 4
#define ignition_sensor A0
boolean ignition_status = false;

void gpsdump(TinyGPS &gps);
void printFloat(double f, int digits = 2);

void setup()
{
  sim800l.begin(9600);
  mySerial.begin(9600);
  Serial.begin(9600);   
  delay(1000);

  
  pinMode(ignition_switch, OUTPUT);
  pinMode(ignition_sensor, INPUT);
}


 
void loop()
{
  ignition_status = getIgnitionStatus();
  Serial.println(ignition_status);
  if(ignition_status==1){
      track();
      digitalWrite(ignition_switch,HIGH);
  }
  else{
      Serial.println("no ignition");
      Serial.println(digitalRead(ignition_switch));
      digitalWrite(ignition_switch,LOW);
    }
    delay(500);
//  String text = ReadSMS();
//  Serial.println(text);
//  delay(1000);
}


boolean getIgnitionStatus()
{
  Serial.println("inside ignition sensor");
  float val = 0;
  for (int i = 1; i <= 10; i++)
  {
    val = val + analogRead(ignition_sensor);
    Serial.println(analogRead(ignition_sensor));
  }
  val = val / 100;
  Serial.println(val);
  if (val > 90)
  {
    return true;
  }
  else if (val < 50)
  {
    return false;
  }
}



void track(){
  bool newdata = false;
  unsigned long start = millis();
  // Every 5 seconds we print an update
  while (millis() - start < 5000) 
  {
    if (mySerial.available()) 
    
    {
      char c = mySerial.read();
//      Serial.print(c);  // uncomment to see raw GPS data
      if (gps.encode(c)) 
      {
        newdata = true;
        break;  // uncomment to print new data immediately!
      }
    }

    else
    {
      Serial.println("gps pai nai");

    }
  }
  
  if (newdata) 
  {
    Serial.println("paiseee");
    gpsdump(gps);
    Serial.println();
  }
  
}


void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  Serial.println("Sending SMS...");
  Serial.println("lattitude");
  Serial.println(flat);

  Serial.println("lon");
  Serial.println(flon);
//  String text = ReadSMS();
//  Serial.println(text);
  sim800l.print("AT+CMGF=1\r");
  delay(100);
  sim800l.print("AT+CMGS=\"+8801738131110\"\r");//
  delay(500);
  sim800l.print("Some one is trying to break into your car!!!!\n");

  sim800l.print("http://maps.google.com/maps?q=loc:");
  sim800l.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
  sim800l.print(","); 
  sim800l.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  sim800l.print((char)26);
  delay(500);
  sim800l.println();
  Serial.println("Text Sent.");
  delay(10000);
}


String ReadSMS(){
//  delay(5000);
  
  // Query for new SMS

  String smsContent;
  Serial.println("Start Reading SMS");
 
  sim800l.println("AT+CMGL=1");
  sim800l.println("AT+CNMI=2,1,0,0,0");
  sim800l.println("AT+CNMI=1,2,0,0,0"); // Decides how newly arrived SMS messages should be handled

  Serial.println(sim800l.available());
  
  // Wait for response
  delay(1000);
  
  // Read response
  while (1) {
    delay(1000);
    Serial.println("Swaiting f0r sms");
    if(sim800l.available()){
      
      Serial.println("SMS paisiiiiiii");
      delay(5000);

      String response = sim800l.readStringUntil('\n');
      Serial.println(response);
    
      // Extract message content
      if (response.startsWith("+CMGL:")) {
      // Read the next line which contains the SMS content
      smsContent = sim800l.readStringUntil('\n');
      Serial.println("SMS Content: " + smsContent);
      return smsContent;
      

    }
  }
    }

}

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.00) 
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.9990, 2) prints as "2.000"
  double rounding = 0.50;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.00;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 00)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0) 
  {
    remainder *= 10.00;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint;
  }
}
