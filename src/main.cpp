/*
  LoRa Duplex communication wth callback

  Sends a message every half second, and uses callback
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Note: while sending, LoRa radio is not listening for incoming messages.
  Note2: when using the callback method, you can't use any of the Stream
  functions that rely on the timeout, such as readString, parseInt(), etc.

  created 28 April 2017
  by Tom Igoe
*/
#include "Arduino.h"
#include <SPI.h> // include libraries
#include <LoRa.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <queue>
#include "SSD1306.h"
//#include <string>
#include <iostream>
#include <sstream>

 
#define BAND 433E6 //915E6

const uint8_t oledAddress = 0x3c;

const int oledSdaPin = 4;
const int oledSclPin = 15;
const int oledRst = 16;

const int loraCsPin = 18;     // LoRa radio chip select
const int loraResetPin = 14;  // LoRa radio reset
const int loraIrqPin = 26;    // change for your board; must be a hardware interrupt pin
const int loraSckPin = 5;
const int loraMisoPin = 19;
const int loraMosiPin = 27;


const uint8_t CMD_UP = 1;
const uint8_t CMD_DOWN = 2;
const uint8_t CMD_ENABLE_WIFI = 3;
const uint8_t CMD_DISABLE_WIFI = 4;
const uint8_t CMD_OTA = 5;
const uint8_t CMD_STATUS = 6;


//B0 ... destination address
//B1 ... destination subnet
//B2 ... sender address
//B3 ... message ID
//B4 ... payload length
//B5..N ... payload

//P0 ... command
//  01 ... up
//  02 ... down
//  03 ... enable wifi
//  04 ... disable wifi
//  05 ... ota
//  06 ... status
//  07 ... status_reply

//P1 ... status field
//  01 ... battery voltage
//  02 ...


const byte broadcastAddressing = 0xFF;
const byte subnetAddressing = 0xFE;
String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte myAddress = 0x17;    // address of this device
byte mySubnet = 0x01;     // address of this device
byte destAddress = 0xFF;  // destination to send to
byte destSubnet = 0x00;   // destination to send to
long lastSendTime = 0;    // last send time
int interval = 2000;      // interval between sends

const char *filename = "/config.txt"; // <- SD library uses 8.3 filenames


struct Config
{
  char hostname[64];
  uint8_t address;
  uint8_t subnet;
};

struct Packet {
  int destAddress;       // destAddress address
  int destSubnet;        // destSubnet address
  byte senderAddress;  // senderAddress address
  byte msgId;  // incoming msg ID
  byte payloadLength;  // incoming msg length
  byte *payload;
  
  Packet() {
    payload = new byte[128];
  } 

  String toString() {
    return String((char *)payload);
  }

};


std::queue<uint8_t> rxCmdQueue;
std::queue<uint8_t> txCmdQueue;

Config config;                        // <- global configuration object

SSD1306 display(oledAddress, oledSdaPin, oledSclPin);


void loadConfiguration(const char *filename, Config &config)
{
  // Open file for reading
  File file = SPIFFS.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  config.address = doc["address"] | 0x00;
  config.subnet = doc["subnet"] | 0x00;
  strlcpy(config.hostname,                 // <- destination
          doc["hostname"] | "example.com", // <- source
          sizeof(config.hostname));        // <- destination's capacity

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

void saveConfiguration(const char *filename, const Config &config)
{
  // Delete existing file, otherwise the configuration is appended to the file
  SPIFFS.remove(filename);

  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc["hostname"] = config.hostname;
  doc["address"] = config.address;
  doc["subnet"] = config.subnet;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}



void sendPacket(const Packet &packet) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(packet.destAddress);       // add destination address
  LoRa.write(packet.destSubnet);        // add destination subnet
  LoRa.write(packet.senderAddress);     // add senderAddress address
  LoRa.write(packet.msgId);             // add message ID
  LoRa.write(packet.payloadLength); // add payload length
  LoRa.write(packet.payload, packet.payloadLength);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID

}
 

// void sendMessage(String outgoing)
// {
//   LoRa.beginPacket();            // start packet
//   LoRa.write(destAddress);       // add destination address
//   LoRa.write(destSubnet);      // add senderAddress address
//   LoRa.write(myAddress);      // add senderAddress address
//   LoRa.write(msgCount);          // add message ID
//   LoRa.write(outgoing.length()); // add payload length
//   LoRa.print(outgoing);          // add payload
//   LoRa.endPacket();              // finish packet and send it
//   msgCount++;                    // increment message ID
// }





Packet rxPacket;
Packet txPacket;

void onReceive(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  rxPacket.destAddress = LoRa.read();       // destAddress address
  rxPacket.destSubnet = LoRa.read();        // destSubnet address
  rxPacket.senderAddress = LoRa.read();         // senderAddress address
  rxPacket.msgId = LoRa.read();  // incoming msg ID
  rxPacket.payloadLength = LoRa.read(); // incoming msg length
  
  int  idx = 0;
  while (LoRa.available())
  {                                // can't use readString() in callback, so
    rxPacket.payload[idx++] = LoRa.read(); // add bytes one by one
  }
  rxPacket.payload[idx] = '\0';


  String incoming; // payload of packet
  incoming = rxPacket.toString();

  Serial.print("Incoming length: ");
  Serial.println(rxPacket.payloadLength);
  Serial.print("Length incoming: ");
  Serial.println(incoming.length());
  Serial.print("Message: ");
  Serial.println(rxPacket.toString());
  if (rxPacket.payloadLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return; // skip rest of function
  }

  // if the destAddress isn't this device or broadcast,
  if ((rxPacket.destAddress != myAddress) && (rxPacket.destAddress != broadcastAddressing))
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  } 

  if ((rxPacket.destSubnet != mySubnet) && (rxPacket.destAddress != subnetAddressing))
  {
    Serial.println("This message is not for me.");
    return; // skip rest of function
  } 


  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(rxPacket.senderAddress, HEX));
  Serial.println("Sent to: 0x" + String(rxPacket.destAddress, HEX));
  Serial.println("Message ID: " + String(rxPacket.msgId));
  Serial.println("Message length: " + String(rxPacket.payloadLength));
  Serial.println("Message: " + incoming);
  Serial.println(rxPacket.toString());
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));
  

  Serial.print("Received packet. ");
  // String line = "Received packet ";// + String(packetSize);
  // display.clear();
  // display.setFont(ArialMT_Plain_10);
  // display.drawString(3, 0, line);
  // //display.drawString(20, 22, "S: " + incoming);
  // display.display();
  Serial.println("Pushing command!");
  rxCmdQueue.push(rxPacket.payload[0]);
  txCmdQueue.push(rxPacket.payload[0]);

}


void publish() {


}

void processTxCommand() {

  if (!txCmdQueue.empty()) {
    Serial.println("Command queue processing");
    uint8_t cmd = txCmdQueue.front();
    txCmdQueue.pop();
    
    std::stringstream ss;
    ss << "CmdOk:" << int(cmd) << std::endl;

    std::string test = ss.str();
    String message = test.c_str(); // send a message
    

    txPacket.destAddress = destAddress;       // destAddress address
    txPacket.destSubnet = destSubnet;        // destSubnet address
    txPacket.senderAddress = myAddress;         // senderAddress address
    txPacket.msgId = msgCount;  // incoming msg ID
    txPacket.payloadLength = message.length(); // incoming msg length
    strcpy((char *)txPacket.payload, message.c_str());

    // sendMessage(message);
    sendPacket(txPacket);
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(3, 5, F("Rcvr: packet "));
    display.display();
    Serial.println("Sending " + message);
  }
}

void processRxCommand() {

  if (!rxCmdQueue.empty()) {
    Serial.println("Command queue processing");
    uint8_t cmd = rxCmdQueue.front();
    rxCmdQueue.pop();
    String msg;
    switch (cmd)
    {
    case CMD_UP:
      msg = ("Blinds up!");
      break;
    case CMD_DOWN:
      msg = ("Blinds down!");
      break;
    case CMD_ENABLE_WIFI:
      msg = ("Wifi on!");
      break;
    case CMD_DISABLE_WIFI:
      msg = ("Wifi off!");
      break;
    case CMD_OTA:
      msg = ("Ota enabled!");
      break;
    case CMD_STATUS:
      msg = ("Status!");
      break;
    default:
      msg = ("Not a valid command!");
    }
    Serial.print("Command: "); 
    Serial.println(msg); 
    display.drawString(28, 22, msg);
    display.display();
  }


}



void setupLora() {

  LoRa.setPins(loraCsPin, loraResetPin, loraIrqPin); // set CS, reset, IRQ pin

  if (!LoRa.begin(BAND))
  { // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true); // if failed, do nothing
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSyncWord(0x12);

  LoRa.onReceive(onReceive);
  LoRa.receive();


}

void setupDisplay() {

  pinMode(oledRst,OUTPUT);
  digitalWrite(oledRst, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(oledRst, HIGH);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Receiver"); 
  display.display();
}




void setup()
{
  Serial.begin(115200); // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex with callback");

  if (!SPIFFS.begin())
  {
    Serial.println("Cannot access SPIFFS");
    return;
  }

  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);
  myAddress = config.address;
  mySubnet = config.subnet;
  Serial.print(F("Set address to..."));
  Serial.println(myAddress);

  const int ledPin = 25;

  pinMode(ledPin,OUTPUT); //Send success, LED will bright 1 second


  setupDisplay();


  SPI.begin(loraSckPin, loraMisoPin, loraMosiPin, loraCsPin);
  // override the default CS, reset, and IRQ pins (optional)

  setupLora();



  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);

}

void loop()
{
  if (millis() - lastSendTime > interval)
  {
    processTxCommand();
    lastSendTime = millis();        // timestamp the message
    interval = random(2000) + 1000; // 2-3 seconds
    LoRa.receive();                 // go back into receive mode
  }

  processRxCommand();

}

