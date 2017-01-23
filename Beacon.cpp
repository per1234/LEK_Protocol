#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>

#include "Beacon.h"
#include "LEK_Protocol.h"
#include "LEK_Protocol_Definitions.h"

Beacon::Beacon(uint8_t type, String node_name, uint8_t network_address, uint32_t uptime_ticks, uint8_t software_version, uint32_t receive_time, int8_t rssi){
	/* Beacons will remain in the list for 2 minutes. If they're not heard from beyond that, they disappear. */
	lifetime = LEK_DEFAULT_BEACON_LIFETIME;
}

Beacon::~Beacon(){
	
}

