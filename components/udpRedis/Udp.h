// Server side implementation of UDP client-server model
#ifndef UDP_H
#define UDP_H
#include <limero.h>
#include <lwip/netdb.h>
#include <string.h>
#include <sys/param.h>

#include <string>
#include <vector>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"

#define PORT 9001
#define UDP_MAX_SIZE 1500

typedef std::vector<uint8_t> Bytes;

struct UdpAddress {
  in_addr_t ip;
  uint16_t port;
  bool operator==(UdpAddress &other) {
    return memcmp(&other.ip, &ip, sizeof ip) == 0 && other.port == port;
  }
  static bool fromUri(UdpAddress &, std::string);
  /* bool operator()(const UdpAddress &lhs, const UdpAddress &rhs) const {
     return false;
   }*/
  bool operator<(const UdpAddress &other) const {
    return memcmp(&other.ip, &ip, sizeof ip) < 0;
  }
  /*UdpAddress& operator=(const UdpAddress& rhs){
    port = rhs.port;
    memcpy(&ip,&rhs.ip,sizeof( in_addr_t));
    return *this;
  }*/
  std::string toString() const;
};

struct UdpMsg {
 public:
  UdpAddress src;
  UdpAddress dst;
  Bytes message;
  void dstIpString(const char *ip) { dst.ip = inet_addr(ip); }
  void dstPort(uint16_t port) { dst.port = htons(port); }
};

class Udp : public Actor {
  uint16_t _myPort = 1883;
  int _sockfd;
  Bytes _rxdBuffer;
  ValueFlow<Bytes> _txd;
  ValueFlow<Bytes> _rxd;
  ValueFlow<bool> _wifiConnected;
  UdpAddress _dst;
  UdpMsg _udpMsg;
  TimerSource _recvTimer;
  UdpMsg _txdMsg;
  UdpMsg _rxdMsg;

 public:
  Udp(Thread &thr);
  Sink<Bytes> &txd() { return _txd; }
  Source<Bytes> &rxd() { return _rxd; }
  Sink<bool> &wifiConnected() { return _wifiConnected; }
  void dst(const char *);
  void port(uint16_t port) { _myPort = port; }
  int init();
  int deInit();
  int receive(UdpMsg &);
  int send(const UdpMsg &);
  int fd() { return _sockfd; };
};
#endif
