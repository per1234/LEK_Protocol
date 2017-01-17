#ifndef _LEK_PROTOCOL_H_
#define _LEK_PROTOCOL_H_

#include <RH_RF95.h>
#include <elapsedMillis.h>
#include <QueueList.h>
#include "LEK_Protocol_Definitions.h"

/*
Style Guide: 
Typedefs, and Enums: Leading capital CamelCase. Enums start with a lowercase k.
Defines: All capitals, underscores as spaces. All defines are prefixed with LEK_.
Functions: Leading lowercase camelCase.
Vars: All lowercase, underscores as spaces.
Global vars: Same as vars, but with a leading underscore.
*/

enum DeviceMode {
    kGATEWAY_SLAVE,
    kGATEWAY_CONSOLE,
    kRECEIVER_DEPLOYED,
    kRECEIVER_UNPAIRED
};

enum ScriptStorageLocation {
    kSCRIPT_PERM,
    kSCRIPT_TEMP
};

enum GatewayConsoleState
{
    kCONSOLE_STATE_MAIN_MENU,
    kCONSOLE_STATE_SCANNING,
    kCONSOLE_STATE_PAIRING,
    kCONSOLE_STATE_RECEIVING,
    kCONSOLE_STATE_CLOSED
};

enum ReceiverReceiveState
{
    kRECEIVER_STATE_NOMINAL,
    kRECEIVER_STATE_SCRIPT_PACK,

};

enum UtilityCallbackDefinition 
{
  	kE_IO_TRIGGER     = 1,
  	kE_SEND_PACKET    = 2,
  	/* The idea here is that you can pack any commands you want to call from events into 
  	a script, and then select those indexes for execution in the callback */
  	kE_SCRIPT_EXECUTE = 3,
  	kE_INDEX_EXECUTE  = 4
};

/* This lame thing here sets the size of our registered event array. We can't count this
enum because we need the event types to be definable outside of the scope of compilation. */
#define LEK_NUMBER_OF_REGISTERABLE_EVENTS 8

enum EventType
{
    kEVENT_GATEWAY_BEACON_WITH_ID         = 0xC0,
    kEVENT_GATEWAY_BEACON                 = 0xC1,
    kEVENT_POWER_LOSS                     = 0xC2,
    kEVENT_RECEIVER_BEACON                = 0xC3,
    kEVENT_RECEIVER_BEACON_WITH_ID        = 0xC4,
    kEVENT_MESSAGE_AUTHENTICATION_FAILURE = 0xC5,
    kEVENT_UPTIME_TICKS                   = 0xC6,
    kEVENT_RTC_TIME                       = 0xC7,
    kEVENT_SERIAL_MESSAGE_RECEIVED        = 0xC8
};

enum ErrorType
{
    kERROR_NO_LINE_AT_INDEX               = 0x05,
    kERROR_NO_SCRIPT_AT_INDEX             = 0x06,
    kERROR_INCOMPLETE_TRANSACTION         = 0x07,
    kERROR_CONFIGURATION_NOT_SET          = 0x08,
    kERROR_BAD_MESSAGE_FORMAT             = 0x09
};

typedef uint8_t (*EventCallbackFunction)(void);
typedef uint8_t (*ScriptCallFunction)(void);

typedef struct {
    uint8_t storage_position;
    uint8_t lines;
    uint8_t line_sizes[LEK_MAX_SCRIPT_LINES];
    uint8_t line_commands[LEK_MAX_SCRIPT_LINES];
} ScriptMap;

/* TODO: This is very C... And, it could probably be made more C++y. */
String getNodeName(uint8_t *packet, uint8_t node_name_length, uint8_t index);
uint8_t getUnsigned8(uint8_t *packet, uint8_t index);
int8_t getSigned8(uint8_t *packet, uint8_t index);
int16_t getSigned16(uint8_t *packet, uint8_t index);
uint16_t getUnsigned16(uint8_t *packet, uint8_t index);
int32_t getSigned32(uint8_t *packet, uint8_t index);
uint32_t getUnsigned32(uint8_t *packet, uint8_t index);

uint8_t putNodeName(uint8_t packet[], uint8_t index, const char *node_name);
void putSigned8(uint8_t packet[], uint8_t index, int8_t su8);
void putUnsigned8(uint8_t packet[], uint8_t index, uint8_t ui8);
void putSigned16(uint8_t packet[], uint8_t index, int16_t su16);
void putUnsigned16(uint8_t packet[], uint8_t index, uint16_t ui16);
void putSigned32(uint8_t packet[], uint8_t index, int32_t su32);
void putUnsigned32(uint8_t packet[], uint8_t index, uint32_t ui32);

void signalFaultTrap();
void signalFastBlink();

uint8_t *decryptIncomingPacket();
uint8_t *encryptOutgoingPacket(uint8_t packet[]);

/* Keyboard and Mouse Reduction Functions -- Refactor: Does not adhere to style guide. Needs some cleanup
and modularization. Maybe, roll into an Attack class? */
void k(int key);
void mod(int mod, int key);
void ctrl(int key);
void cmd(int key);
void shift(int key);
void execute_test_usb_driveby_windows();
void execute_test_usb_driveby_osx();
void openappWindows(String app);
void execute_usb_driveby_windows();
void executeUsbDriveByOsX();
void typeln(String chars);
void openappMacOs(String app);
void clickOutOfLittleSnitch();
void pwnLittleSnitch(bool _pwn_little_snitch_with_mouse);

class LEK_Protocol {
 public:
  LEK_Protocol(DeviceMode user_mode, uint8_t user_network_address, 
    uint64_t user_uuid, const char* user_node_name, bool user_beaconing);
  ~LEK_Protocol();
  void begin();
  void spin();

  void displayConsole();
  void updateConsoleState(GatewayConsoleState state);
  void clearTerminal();

  void reloadParametersFromEEPROM();

  void duckyParse(const char* command_string);
  uint8_t executeScriptIndex(ScriptStorageLocation stored, uint8_t storage_index, uint8_t script_index);
  uint8_t executeScript(ScriptStorageLocation stored, uint8_t storage_index, uint8_t from_index);
  uint8_t storeScript(ScriptStorageLocation store_in, uint8_t storage_index, uint8_t script_index, const char *line);
  uint8_t commitScriptToEeprom(const char *script, uint8_t storage_index);
  uint8_t retriveScriptFromEeprom();
  uint8_t deleteScript(ScriptStorageLocation store_in, uint8_t storage_index);

  uint8_t registerEvent(uint8_t (*callback_function)(void), uint8_t event_slot);
  uint8_t gatewayBeaconEvent();
  uint8_t gatewayBeaconWithIdEvent();
  uint8_t powerUpEvent();
  uint8_t receiverBeaconEvent();
  uint8_t receiverBeaconWithIdEvent();
  uint8_t messageAuthenticationFailureEvent();
  uint8_t uptimeTicksEvent();
  uint8_t rtcTicksEvent();
  uint8_t serialInterfaceReceiveEvent();

  uint8_t *createEventPollConfigurationResponse(uint8_t message_sequence);
  uint8_t *createScheduleResponse(uint8_t message_sequence);
  uint8_t *createSetTime(uint8_t message_sequence, uint32_t current_time);
  uint8_t *createScriptPackConclude(uint8_t message_sequence, uint8_t initial_sequence);
  uint8_t *createAcknowledgement(bool nack, uint8_t message_sequence, uint8_t error_code);
  uint8_t *createRequestResponse(uint8_t request_sequence, const char* _node_name, uint8_t _network_address, uint32_t _uptime_ticks);
  uint8_t *createBeacon(DeviceMode _mode, const char* _node_name, uint8_t _network_address, uint32_t _uptime_ticks, uint8_t _software_version);
  uint8_t *createSendKey(uint16_t message_sequence, uint8_t key);
  uint8_t *createSendModKey(uint16_t message_sequence, uint8_t mod_key, uint8_t key);
  uint8_t *createExecuteBakedRoutine(uint16_t message_sequence, uint8_t routine_index);
  int determinePacketLength(const uint8_t *packet, size_t absolute_packet_size);
  bool validatePacket(const uint8_t *packet, size_t absolute_packet_size)
  void printPacketASCII(const uint8_t *packet, size_t absolute_packet_size);
  uint32_t generateMessageSalt();
  uint32_t generateCurrentTime();

  void manageTransactions();
  void openTransaction();
  void closeTransaction();

  /* This could use a refactoring to something a bit less cumbersome... */
  void routePacketToHandler(const uint8_t *packet, size_t absolute_packet_size);
  void handlerBeaconPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerGatewayStatusResponsePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerAckPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerNackPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerScriptPackConcludePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerScheduleResponsePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerEventPollConfigurationResponsePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerRevShellCompletePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerKeyPressPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerModifierPressPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerLinePressPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerMouseMovePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerSetTimePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerPackLinePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerPackScriptPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerScheduleScriptPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerStatusRequestPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerSetEventTriggerPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerClearLoadsPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerResetPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerScuttlePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerScheduleRequestPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerExecuteBakedRoutinePacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerNopPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerUsbMuxCtlPacket(const uint8_t *packet, size_t absolute_packet_size);
  void handlerUsbSlavePowerCtlPacket(const uint8_t *packet, size_t absolute_packet_size);

  void setSystemTime(byte seconds, byte minutes, byte hours, byte day, byte month, byte year);
  uint64_t getSystemTime(void);

  /* This ticker will overflow in 49.1 days if it's not cycled off into another variable at another rate. */
  elapsedMillis uptime_ticks_actual;
  elapsedMillis beacon_ticks;

 private:
  DeviceMode _mode;
  uint8_t _global_sequence;
  uint8_t _network_address;
  uint8_t _scripts_loaded;
  uint64_t _uuid;
  char _node_name[LEK_MAX_NODE_NAME+1];
  bool _beaconing;
  bool _radio_state;

  /* Statistics */
  uint16_t _tick_rate;
  uint64_t _uptime_ticks;
  int8_t _last_rssi;
  uint16_t _count_of_failed_packet_receives;
  
  GatewayConsoleState _console_state;
  GatewayConsoleState _last_console_state;
  String _console_buffer;
  
  EventCallbackFunction _event_callbacks[LEK_NUMBER_OF_EVENT_CALLBACKS];
  EventType _registered_events[LEK_NUMBER_OF_REGISTERABLE_EVENTS];
  
  RH_RF95 *_rf_module;
  QueueList<uint8_t*> _packet_buffer;
  /* Protocol Controls */
  uint8_t _beacons_nearby[LEK_MAX_BEACONS_NEARBY];
  uint8_t _count_of_beacons_nearby;

  /* Statically allocated buffers, you say? */
  char _temp_script[LEK_MAX_SCRIPT_SIZE];

  void rfmReset();
  void rfmSetup();
  void rfmSendPacket(const uint8_t *packet, uint8_t packet_length);
  void rfmReadPacket();
  /* This function puts the packet into the packet queue */
  bool rfmReadRadioState();
  void rfmSetRadioState(bool radio_state)
};

#endif /* _LEK_PROTOCOL_H_ */
