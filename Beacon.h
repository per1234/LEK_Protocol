#ifndef _BEACON_H_
#define _BEACON_H_

#include <Arduino.h>

/* I generally dislike single word nomenclature for classes, but here we are... */

class Beacon {
 public:
  Beacon(uint8_t type, String node_name, uint8_t network_address, uint32_t uptime_ticks, uint8_t software_version, uint32_t receive_time, int8_t rssi);
  ~Beacon();
  uint8_t type;
  String node_name;
  uint8_t network_address;
  uint32_t uptime_ticks;
  uint8_t software_version;
  uint32_t receive_time;
  uint32_t lifetime;
  int8_t rssi;
 private:
};

#endif /* _BEACON_H_ */