
#include <Log.h>
#include <StringUtility.h>
#include <Udp.h>
#include <arpa/inet.h>
#include <errno.h>   //For errno - the error number
#include <netdb.h>   //hostent
//#include <stdio.h>   //printf
//#include <stdlib.h>  //for exit(0);
#include <string.h>  //memset
#include <sys/socket.h>

Thread udpRcvThread("udpRcvThread");
Thread udpSndThread("udpSndThread");

Udp::Udp(Thread &thr)
    : Actor(thr), _recvTimer(udpRcvThread, 1000, true, "recvTimer") {
  _rxdBuffer.resize(UDP_MAX_SIZE);
  _txd.async(udpSndThread);
  _rxd.async(thr);
  _txd >> [&](const Bytes &in) {
    INFO("UDP TXD[%d] to %s ", in.size(), _dst.toString().c_str());
    _txdMsg.dst = _dst;
    _txdMsg.message = in;
    send(_txdMsg);
  };
  _recvTimer >> [this](const TimerMsg &) {
    while (_wifiConnected()) {
      while (receive(_rxdMsg) == 0) _rxd.on(_rxdMsg.message);
    }
  };
  _wifiConnected >> [this](const bool &in) {
    if (in) {
      init();
    } else {
      deInit();
    }
  };
  udpRcvThread.start();
  udpSndThread.start();
}

void Udp::dst(const char *ip) { UdpAddress::fromUri(_dst, ip); }

int Udp::init() {
  struct sockaddr_in servaddr;
  if ((_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    WARN("socket creation failed %d : %s", errno, strerror(errno));
    return (errno);
  }

  int optval = 1;
  setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval,
             sizeof(int));
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;  // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(_myPort);
  if (bind(_sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    WARN("bind failed %d : %s", errno, strerror(errno));
    return (errno);
  }

  INFO("UDP listening port:%d socket:%d", _myPort, _sockfd);
  return 0;
}

int Udp::deInit() {
  int rc = close(_sockfd);
  if (rc) WARN("close failed");
  return rc;
}

int Udp::receive(UdpMsg &rxd) {
  struct sockaddr_in clientaddr;
  memset(&clientaddr, 0, sizeof(clientaddr));

  socklen_t len = sizeof(clientaddr);  // len is value/resuslt
  clientaddr.sin_family = AF_INET;
  clientaddr.sin_port = htons(_myPort);
  clientaddr.sin_addr.s_addr = INADDR_ANY;
  rxd.message.resize(1500);

  int rc = recvfrom(_sockfd, (char *)_rxdBuffer.data(), UDP_MAX_SIZE,
                    MSG_WAITALL, (struct sockaddr *)&clientaddr, &len);

  if (rc >= 0) {
    rxd.message.clear();
    for (uint32_t i = 0; i < rc; i++) rxd.message.push_back(_rxdBuffer[i]);
    rxd.src.ip = clientaddr.sin_addr.s_addr;
    rxd.src.port = ntohs(clientaddr.sin_port);
    rxd.dst.ip = INADDR_ANY;
    rxd.dst.port = _myPort;
    INFO(" received [%d] from %s to %s  ", rc, rxd.src.toString().c_str(),
         rxd.dst.toString().c_str());
    return 0;
  } else {
    return errno;
  }
}

int Udp::send(const UdpMsg &udpMsg) {
  struct sockaddr_in server;
  server.sin_family = AF_INET;
  server.sin_port = htons(udpMsg.dst.port);
  server.sin_addr.s_addr = udpMsg.dst.ip;

  /* INFO("TXD UDP => %s : %s ", udpMsg.dst.toString().c_str(),
        hexDump(udpMsg.message).c_str());*/

  int rc = sendto(_sockfd, udpMsg.message.data(), udpMsg.message.size(), 0,
                  (const struct sockaddr *)&server, sizeof(server));
  if (rc < 0) return errno;
  return 0;
}

bool getInetAddr(in_addr_t &addr, std::string &hostname) {
  BZERO(addr);
  struct addrinfo hints, *servinfo, *p;
  //  struct sockaddr_in *h;
  int rv;

  BZERO(hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  if ((rv = getaddrinfo(hostname.c_str(), 0, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", strerror(rv));
    return false;
  }

  // loop through all the results and connect to the first we can
  for (p = servinfo; p != NULL; p = p->ai_next) {
    if (p->ai_addr->sa_family == AF_INET) {
      sockaddr *sa = p->ai_addr;
      addr = ((sockaddr_in *)sa)->sin_addr.s_addr;
      freeaddrinfo(servinfo);  // all done with this structure
      return true;
    }
  }
  freeaddrinfo(servinfo);  // all done with this structure
  return false;
}

bool getNetPort(uint16_t &x, const std::string &s) {
  x = 0;
  for (char const &ch : s) {
    if (std::isdigit(ch)) {
      x *= 10;
      x += ch - '0';
    } else {
      return false;
    }
  }
  return true;
}

bool UdpAddress::fromUri(UdpAddress &udpAddress, std::string uri) {
  auto parts = split(uri, ':');

  return parts.size() == 2 && getInetAddr(udpAddress.ip, parts[0]) &&
         getNetPort(udpAddress.port, parts[1]);
}

std::string UdpAddress::toString() const {
  char charBuffer[100];
  const char *ipString =
      inet_ntop(AF_INET, &ip, charBuffer, sizeof(charBuffer));
  std::string url = ipString == 0 ? "UNKNOWN" : ipString;
  url += ":";
  url += std::to_string(port);
  return url;
}