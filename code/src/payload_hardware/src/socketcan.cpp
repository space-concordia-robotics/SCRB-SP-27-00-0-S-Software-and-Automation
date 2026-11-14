#include "payload_hardware/socketcan.hpp"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

namespace payload_hardware {
namespace can {

bool SocketCan::open(const std::string& ifname, std::string* err) {
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) { if (err) *err = std::string("socket(): ") + strerror(errno); return false; }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    if (err) *err = "no such CAN interface '" + ifname + "' (" + strerror(errno) + ")";
    close();
    return false;
  }

  // Only deliver frames that carry OUR arbitration id (29-bit extended), plus error frames.
  struct can_filter filt;
  filt.can_id = kArbitrationId | CAN_EFF_FLAG;
  filt.can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt));
  can_err_mask_t errmask = CAN_ERR_MASK;                      // ask the driver to report bus errors
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errmask, sizeof(errmask));
  int no_echo = 0;                                            // never receive our own transmissions
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &no_echo, sizeof(no_echo));

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    if (err) *err = std::string("bind(): ") + strerror(errno);
    close();
    return false;
  }
  return true;
}

void SocketCan::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SocketCan::send(const Frame& f) {
  if (fd_ < 0) return false;
  struct can_frame cf;
  std::memset(&cf, 0, sizeof(cf));
  cf.can_id = (f.id & CAN_EFF_MASK) | CAN_EFF_FLAG;           // extended (29-bit) frame
  cf.can_dlc = f.dlc > 8 ? 8 : f.dlc;
  std::memcpy(cf.data, f.data.data(), cf.can_dlc);
  const ssize_t n = ::send(fd_, &cf, sizeof(cf), MSG_DONTWAIT);  // never block the control loop
  return n == static_cast<ssize_t>(sizeof(cf));
}

RecvStatus SocketCan::recv(Frame* f, int timeout_ms) {
  if (fd_ < 0) return RecvStatus::kClosed;
  struct pollfd p;
  p.fd = fd_;
  p.events = POLLIN;
  const int r = ::poll(&p, 1, timeout_ms);
  if (r == 0) return RecvStatus::kTimeout;
  if (r < 0) return errno == EINTR ? RecvStatus::kTimeout : RecvStatus::kClosed;

  struct can_frame cf;
  const ssize_t n = ::recv(fd_, &cf, sizeof(cf), 0);
  if (n != static_cast<ssize_t>(sizeof(cf))) return RecvStatus::kClosed;
  if (cf.can_id & CAN_ERR_FLAG) return RecvStatus::kBusError;  // bus-off, controller problem, ...
  f->id = cf.can_id & CAN_EFF_MASK;
  f->dlc = cf.can_dlc;
  std::memcpy(f->data.data(), cf.data, 8);
  return RecvStatus::kFrame;
}

}  // namespace can
}  // namespace payload_hardware
