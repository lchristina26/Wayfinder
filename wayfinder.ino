#include <DW1000.h>
#include <SoftwareSerial.h>

/*
 * Message Syntax:
 *  INIT        ===> 999, 1, [<clientAddress>]
 *  COMFIRM     ===> <NodeAddress>, 2, [<clientAddress>, <destination>]
 *  CANIDATE    ===> <clientAddress>, 3, [<nodeAddress>]
 *  NAVIGATION  ===> <clientAddress>, 9, [<direction>, <distance>]
 *  
 */

#define myAddr  101  // Address of this node
#define rxPin   4  // Serial input (connects to Emic 2's SOUT pin)
#define txPin   5  // Serial output (connects to Emic 2's SIN pin)
#define ledPin  13  // Most Arduino boards have an on-board LED on this pin
#define DW_rst  9  // Reset pin for UWB
#define DW_cs   10  // Slave select for UWB
#define DW_int  2  // UWB interrupt pin

// set up a new serial port
SoftwareSerial emicSerial =  SoftwareSerial(rxPin, txPin);

String message, args, error;     // Last received message
int command;
int myClient, myClientOrigin, myClientDest, lastVelocity;

typedef struct {
  String direction;
  int distance;
} NavInstruction;

void setup()  // Set up code called once on start-up
{
  // define pin modes
  pinMode(ledPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  
  // set the data rate for the SoftwareSerial port
  emicSerial.begin(9600);

  digitalWrite(ledPin, LOW);  // turn LED off

  // Wait for Emic2 to initialize
  emicSerial.print('\n');             // Send a CR in case the system is already up
  while (emicSerial.read() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it
  delay(10);                          // Short delay
  emicSerial.flush();                 // Flush the receive buffer

  //DWM1000 Initialization
  // DEBUG monitoring
  Serial.begin(9600);
  Serial.println("### DW1000-arduino-receiver-test ###");
  // initialize the driver
  DW1000.begin(DW_int, DW_rst);
  DW1000.select(DW_cs);
  Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(addr);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println("Committed configuration ...");
  // DEBUG chip info and registers pretty printed
  char msg[1024];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(didReceiveUWBMessage);
  DW1000.attachReceiveFailedHandler(onMsgFailure);
  DW1000.attachErrorHandler(onUWBError);
  DW1000.attachSentHandler(handleSent);
  // start reception
  receiver();
}

void speakInstruction(int distance, String inst) {
  while (emicSerial.read() != ":");
  emicSerial.print('S');
  emicSerial.print("In " + distance + " feet, " + inst + ".");
  emicSerial.print("\n");
}

// Successful send IRQ
void handleSent() {
  Serial.println("Message sent successfully.");
}

// Receive error interrupt for DWM1000
void onMsgFailure () {
   Serial.println("DWM1000 message receive error!!");
   error = false;
   DW1000.getData(error);
   Serial.print("Error data is ... "); Serial.println(message);
}

// Error interrupt forDW1000
void onDW1000Error () {
   Serial.println("DWM1000 error!!");
   error = false;
   DW1000.getData(error);
   Serial.print("Error data is ... "); Serial.println(message);
}

// IRQ handler for DWM1000
void didReceiveUWBMessage() {
      // get data as string
      DW1000.getData(message);
      parseCommand();
      Serial.print("Data is ... "); Serial.println(message);
      Serial.print("FP power is [dBm] ... "); Serial.println(DW1000.getFirstPathPower());
      Serial.print("RX power is [dBm] ... "); Serial.println(DW1000.getReceivePower());
      Serial.print("Signal quality is ... "); Serial.println(DW1000.getReceiveQuality());
      message = "";
    }
}

void parseCommand () {
  if (sizeof(message) < 1 ) {
    Serial.println("Received empty messgae");
    return;
  }
  
  // Parse
  addrEnd = message.indexOf(":");
  if (message.substring(0, addrEnd) != myAddr) return;
  commandEnd = message.indexOf("_", addrEnd + 1);
  command = message.substring(addrEnd + 1, commandEnd).toInt();
  args = message.substring(commandEnd + 1);

  switch (command) {
    case 1:
      // Initiate navigation
      // Parse args
      _addr = args;
      Serial.println("***Command: init navigation. From node: " + _addr);
      // Get distance to client, send response to client
      int _dist = getDistanceToClient();
      sendCommand(_addr, 3, _dist);

    case 2:
      // Respond to CONFIRM
      // We're now the guide node for the client
      // Parse args:
      int _pos = args.indexOf(",");
      int _addr = args.substring(0, _pos);
      args.remove(0, _pos);
      myClientDest = args;
      myClient = _addr;
      Serial.println("***Command: confirmation. From client: " + myClient); 
      // Send first direction message!
      NavInstruction _inst = getNavigationInstruction(myClientDestination, myClientOrigin);
      sendCommand(myClient, _inst);
      }
  }
}

void onUWBError () {
  Serial.println("An unknown DWM1000 error ocurred.");
}

void sendCommand (int addr, int command, String args) {
  Serial.println("Beginning command send...");
  Serial.println("    Command: " + command + ": " + args);
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(addr + ":" + command + "_" + args);
  DW1000.startTransmit();
}

void sendCommand(int addr, NavInstruction inst) {
  Serial.println("Beginning command send...");
  Serial.println("    Command: 9:" + inst.direction + "," + inst.distance);
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(addr + ":9_" + inst.direction + "," + inst.distance);
  DW1000.startTransmit();
}

NavInstruction getNavigationInstruction (int destination, int origin) {

}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

int getDistanceToClient () {
  int _dist;
  return _dist;
}

int getRelativeVelocity () {
  int _velocity;
  return _velocity;
}

void loop()  // Main code, to run repeatedly
{
  // Client monitoring
  _dist = getDistanceToClient();
  _velocity = getRelativeVelocity();
  if (
}



