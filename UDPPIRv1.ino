#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "newsteadesp";

//the time we give the sensor to calibrate (10-60 secs according to the datasheet)
int calibrationTime = 60;        

//the time when the sensor outputs a low impulse
long unsigned int lowIn;         

//the amount of milliseconds the sensor has to be low 
//before we assume all motion has stopped
long unsigned int pause = 5000;  

boolean lockLow = true;
boolean takeLowTime;  

int pirPin = 1;    //the digital pin connected to the PIR sensor's output 
//GPIO01 FOR ESP11 board use GPIO02 for  esp01 board

string UDPBuff[1420];
WiFiUDP Udp;
// Multicast declarations
IPAddress ipMulti(192.168.1.255);    // send to all local
unsigned int portMulti = 12345;      // local port to listen on


/////////////////////////////
//SETUP
void setup(){
  initHardware();


  //give the sensor some time to calibrate
  Serial.print("calibrating sensor ");
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);
  
  setupWiFi();
  server.begin();
  //Udp.begin(localPort);
  Udp.beginPacketMulticast(ipMulti, portMulti,WiFi.localIP());

  }

void loop(){

    


  
     if(digitalRead(pirPin) == HIGH){
       digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
       if(lockLow){  
         //makes sure we wait for a transition to LOW before any further output is made:
         lockLow = false;            
         UDPBuff ="---";
         UDPBuff += "motion detected at ";
         UDPBuff += sprintf(millis()/1000);
         UDPBuff += " sec"; 

         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  // change to multicast
         Udp.write(UDPBuff);
         Udp.endPacket();
         delay(50);
         }         
         takeLowTime = true;
       }

     if(digitalRead(pirPin) == LOW){       
       digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state

       if(takeLowTime){
        lowIn = millis();          //save the time of the transition from high to LOW
        takeLowTime = false;       //make sure this is only done at the start of a LOW phase
        }
       //if the sensor is low for more than the given pause, 
       //we assume that no more motion is going to happen
       if(!lockLow && millis() - lowIn > pause){  
           //makes sure this block of code is only executed again after 
           //a new motion sequence has been detected
           lockLow = true;                        
         UDPBuff ="---";
         UDPBuff += "motion ended at ";
         UDPBuff += sprintf(millis()/1000);
         UDPBuff += " sec"; 

         Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());  // change to multicast
         Udp.write(UDPBuff);
         Udp.endPacket();
           delay(50);
           }
       }
    


}

void setupWiFi()
{
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid);

    // wait 10 seconds for connection:
    delay(10000);
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ESP8266" + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

void initHardware()
{
  Serial.begin(9600);
  pinMode(pirPin, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(pirPin, LOW);
  pinMode(pirPin, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);
  // Don't need to set ANALOG_PIN as input, 
  // that's all it can be.
}

