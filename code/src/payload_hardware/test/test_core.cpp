// test_core.cpp -- plain C++ tests (no framework needed): g++ -std=c++17 -Iinclude test/test_core.cpp src/can_core.cpp
#include <algorithm>
#include <cmath>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "payload_hardware/can_core.hpp"

using namespace payload_hardware::can;
using Clock = Core::Clock;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond)                                                                   \
  do {                                                                                \
    if (cond) { ++g_pass; } else { ++g_fail; std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)
#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

// ---- URDF constants, typed in by hand from payload_description.urdf (independent of the generator) ----
static const double kVpsMult = -0.0006992351524728963;       // m of VPS box travel per rad of lead screw
static const double kDrillMult = -0.0005305164769729845;     // m of drill platform travel per rad of lift screw

struct FakeTx : Transmitter {
  std::vector<Frame> sent;
  bool fail = false;
  bool send(const Frame& f) override { if (fail) return false; sent.push_back(f); return true; }
};

static const MessageDef* msg_named(const std::string& n) {
  for (int i = 0; i < kNumMessages; ++i) if (n == kMessages[i].name) return &kMessages[i];
  return nullptr;
}
static int sig_named(const MessageDef* m, const std::string& n) {
  for (int i = 0; i < m->n_signals; ++i) if (n == m->signals[i].name) return i;
  return -1;
}

// Build a frame the way the firmware would send telemetry.
static Frame telem(const std::string& name, const std::map<std::string, int64_t>& fields) {
  const MessageDef* m = msg_named(name);
  uint64_t w = make_header(kDestJetson, m->id);
  for (auto& kv : fields) {
    const SignalDef& s = m->signals[sig_named(m, kv.first)];
    w = put_payload(w, s.bit, s.width, static_cast<uint64_t>(kv.second));
  }
  Frame f; f.id = kArbitrationId; from_word(f, w); return f;
}

// Decode a frame we transmitted (what the firmware would see).
static std::map<std::string, int64_t> decode_cmd(const Frame& f, std::string* name) {
  const uint64_t w = to_word(f);
  CHECK(get_dest(w) == kDestPayload);
  const MessageDef* m = nullptr;
  for (int i = 0; i < kNumMessages; ++i) if (kMessages[i].id == get_msg_id(w)) m = &kMessages[i];
  std::map<std::string, int64_t> out;
  if (!m) return out;
  *name = m->name;
  for (int i = 0; i < m->n_signals; ++i) {
    const SignalDef& s = m->signals[i];
    const uint64_t raw = get_payload(w, s.bit, s.width);
    out[s.name] = s.is_signed ? sign_extend(raw, s.width) : static_cast<int64_t>(raw);
  }
  return out;
}

// A full set of interface names, exactly as the real ros2_control block would declare.
struct Rig {
  std::vector<std::string> sn, cn;
  std::vector<double> st, cmd;
  std::unique_ptr<Core> core;
  FakeTx tx;
  Clock::time_point t0 = Clock::now();

  explicit Rig(uint8_t comp = kCompUrc, int retry_max = 2) {
    for (auto& b : state_bindings()) sn.push_back(b.interface);
    for (const char* d : {Core::kLinkOk, Core::kRxAgeMs, Core::kFaultFlags, Core::kTxCount, Core::kRxCount, Core::kRejectedCount}) sn.push_back(d);
    for (auto& b : command_bindings()) {
      for (auto& v : b.values) cn.push_back(v.interface);
      if (std::string(b.trigger).size()) cn.push_back(b.trigger);
    }
    cn.push_back(Core::kClearFaults);
    // de-duplicate (an interface can appear in several bindings)
    auto dedup = [](std::vector<std::string>& v) { std::vector<std::string> o; for (auto& s : v) if (std::find(o.begin(), o.end(), s) == o.end()) o.push_back(s); v = o; };
    dedup(sn); dedup(cn);
    st.assign(sn.size(), Core::kNaN);
    cmd.assign(cn.size(), Core::kNaN);
    CoreConfig cfg; cfg.competition = comp; cfg.retry_max = retry_max;
    core.reset(new Core(cfg));
    auto rep = core->bind(sn, cn);
    for (auto& e : rep.errors) std::printf("  bind error: %s\n", e.c_str());
    CHECK(rep.errors.empty());
  }
  size_t si(const std::string& n) const { return std::find(sn.begin(), sn.end(), n) - sn.begin(); }
  size_t ci(const std::string& n) const { return std::find(cn.begin(), cn.end(), n) - cn.begin(); }
  Clock::time_point at(double s) const { return t0 + std::chrono::microseconds(static_cast<long long>(s * 1e6)); }
  void rx(const Frame& f, double s) { core->on_rx(f, at(s)); }
  void read(double s) { core->read_states(st.data(), at(s)); }
  void write(double s) { core->write_commands(cmd.data(), st.data(), at(s), tx); }
  double S(const std::string& n) const { return st[si(n)]; }
  double& C(const std::string& n) { return cmd[ci(n)]; }
};

// ------------------------------------------------------------------------------------------
static void test_codec() {
  std::printf("codec\n");
  uint64_t w = 0;
  w = set_field(w, 0, 10, 0x3FF);
  CHECK(w == 0xFFC0000000000000ULL);
  CHECK(get_field(w, 0, 10) == 0x3FF);
  CHECK(get_field(w, 10, 10) == 0);
  w = set_field(0, 61, 3, 5);                       // last three bits
  CHECK(w == 5);
  CHECK(sign_extend(0xFFFF, 16) == -1);
  CHECK(sign_extend(0x4E20, 16) == 20000);
  CHECK(sign_extend(0xB1E0, 16) == -20000);
  uint64_t h = make_header(kDestJetson, 42);
  CHECK(get_dest(h) == 1 && get_msg_id(h) == 42);
  Frame f; from_word(f, 0x0102030405060708ULL);
  CHECK(f.data[0] == 1 && f.data[7] == 8 && to_word(f) == 0x0102030405060708ULL);
}

// Cross-check against vectors produced by the independent Python implementation.
static void test_vectors(const char* path) {
  std::printf("vectors (python <-> c++)\n");
  std::ifstream in(path);
  if (!in) { std::printf("  FAIL cannot open %s\n", path); ++g_fail; return; }
  std::string line; int n = 0;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ls(line);
    std::string name, fields, arrow, hex; int dest;
    ls >> name >> dest >> fields >> arrow >> hex;
    const MessageDef* m = msg_named(name);
    CHECK(m != nullptr);
    uint64_t w = make_header(static_cast<uint8_t>(dest), m->id);
    std::istringstream fs(fields); std::string kv;
    while (std::getline(fs, kv, ',')) {
      const auto eq = kv.find('=');
      const SignalDef& s = m->signals[sig_named(m, kv.substr(0, eq))];
      w = put_payload(w, s.bit, s.width, static_cast<uint64_t>(std::stoll(kv.substr(eq + 1))));
    }
    const uint64_t want = std::stoull(hex, nullptr, 16);
    if (w != want) std::printf("  mismatch %s: c++ %016llx python %016llx\n", name.c_str(), (unsigned long long)w, (unsigned long long)want);
    CHECK(w == want);
    // and decoding the python word gives back the same raw fields
    std::istringstream fs2(fields);
    while (std::getline(fs2, kv, ',')) {
      const auto eq = kv.find('=');
      const SignalDef& s = m->signals[sig_named(m, kv.substr(0, eq))];
      const uint64_t raw = get_payload(want, s.bit, s.width);
      const int64_t sv = s.is_signed ? sign_extend(raw, s.width) : static_cast<int64_t>(raw);
      CHECK(sv == std::stoll(kv.substr(eq + 1)));
    }
    ++n;
  }
  CHECK(n >= 15);
}

static void test_table_sanity() {
  std::printf("generated table\n");
  CHECK(kArbitrationId == 0x20000u);
  CHECK(kNumMessages == 40);
  for (int i = 0; i < kNumMessages; ++i) {            // no field overlaps and everything fits in 57 bits
    uint64_t used = 0;
    for (int j = 0; j < kMessages[i].n_signals; ++j) {
      const SignalDef& s = kMessages[i].signals[j];
      CHECK(s.bit + s.width <= kPayloadBits);
      const uint64_t m = mask_bits(s.width) << s.bit;
      CHECK((used & m) == 0);
      used |= m;
    }
  }
}

static void test_telemetry() {
  std::printf("telemetry decode\n");
  Rig r;
  r.read(0.0);
  CHECK(std::isnan(r.S("vps_screw_rotation/position")));            // nothing received yet
  CHECK(r.S("payload_link/link_ok") == 0.0);

  r.rx(telem("TELEM_URC_GENERAL", {{"distance_from_ground", 50}, {"vps_displacement", 100}}), 0.1);
  r.read(0.12);
  // 100 mm of VPS box travel = 0.1 m / (-0.000699235 m/rad) = -143.01 rad
  CHECK_NEAR(r.S("vps_screw_rotation/position"), 0.1 / kVpsMult, 1e-6);
  CHECK_NEAR(r.S("ground_sensor/distance"), 0.050, 1e-9);
  CHECK(r.S("payload_link/link_ok") == 1.0);
  CHECK(r.S("payload_link/rx_count") == 1.0);

  r.rx(telem("TELEM_URC_DEEP_SAMPLE", {{"vps_rpm", 3}, {"drill_disp", 100}, {"drill_vert_rpm", 7}, {"drill_rot_rpm", 60}, {"plug_state", 1}}), 0.13);
  r.read(0.14);
  // 100 mm of depth = translation -0.1 m = -0.1 / (-0.00053052) rad = +188.5 rad
  CHECK_NEAR(r.S("drill_lift_screw_rotation/position"), -0.1 / kDrillMult, 1e-6);
  CHECK_NEAR(r.S("drill_tool_rotation/velocity"), 60 * 2 * M_PI / 60, 1e-9);   // rpm -> rad/s
  CHECK(r.S("plug/state") == 1.0);

  r.rx(telem("TELEM_URC_ANALYZE", {{"carousel_pos", 3}, {"sas_angle", 90}, {"pump1_status", 1}, {"photodiode", 40000}, {"led_445", 1}}), 0.15);
  r.read(0.16);
  CHECK_NEAR(r.S("carousel_rotation/position"), 3 * 2 * M_PI / 12, 1e-9);
  CHECK_NEAR(r.S("soil_sampler_rotation/position"), M_PI / 2, 1e-9);
  CHECK(r.S("photodiode/counts") == 40000.0);
  CHECK(r.S("pump_1/running") == 1.0 && r.S("led_445/on") == 1.0 && r.S("led_405/on") == 0.0);

  // ERC telemetry arrives while we are configured for URC -> must read NaN
  r.rx(telem("TELEM_ERC_DRILL", {{"drill_dist", 500}, {"load_cell_mass", 2500}}), 0.17);
  r.read(0.18);
  CHECK(std::isnan(r.S("erc_drill/distance")));
  CHECK(std::isnan(r.S("erc_drill/mass")));

  // staleness: no new URC frame for > 0.5 s -> NaN, link down
  r.read(1.0);
  CHECK(std::isnan(r.S("vps_screw_rotation/position")));
  CHECK(r.S("payload_link/link_ok") == 0.0);
  CHECK((static_cast<uint32_t>(r.S("payload_link/fault_flags")) & kFaultRxTimeout) != 0);
}

static void test_erc_mode() {
  std::printf("erc competition mode\n");
  Rig r(kCompErc);
  r.rx(telem("TELEM_ERC_DRILL", {{"drill_dist", 500}, {"load_cell_mass", 2500}}), 0.0);
  r.rx(telem("TELEM_ERC_ROCK", {{"rock_mass", 1234}}), 0.0);
  r.rx(telem("TELEM_ERC_SOIL", {{"soil_mass", 50}, {"temperature", 120}}), 0.0);
  r.rx(telem("TELEM_URC_GENERAL", {{"distance_from_ground", 50}}), 0.0);
  r.read(0.1);
  CHECK_NEAR(r.S("erc_drill/distance"), 0.5, 1e-9);
  CHECK_NEAR(r.S("erc_drill/mass"), 2.5, 1e-9);                      // 2500 g -> 2.5 kg
  CHECK_NEAR(r.S("rock_scale/mass"), 0.1234, 1e-9);                  // 1234 dg -> 0.1234 kg
  CHECK_NEAR(r.S("soil/temperature"), 120, 1e-9);
  CHECK(std::isnan(r.S("ground_sensor/distance")));                  // URC data is NaN in ERC mode
}

static void test_value_commands() {
  std::printf("value commands (send on change)\n");
  Rig r;
  // Link up and the VPS sitting at 0 mm
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 0.0);
  r.read(0.01);

  // 1) Controller activation writes "hold current position" = 0 rad: must NOT be sent.
  r.C("vps_screw_rotation/position") = 0.0;
  r.write(0.02);
  CHECK(r.tx.sent.empty());

  // 2) Command 50 mm of travel: rad = 0.05 / mult  -> one VPS_MOVE with raw 50
  r.C("vps_screw_rotation/position") = 0.05 / kVpsMult;
  r.write(0.03);
  CHECK(r.tx.sent.size() == 1);
  std::string name;
  auto f = decode_cmd(r.tx.sent.back(), &name);
  CHECK(name == "VPS_MOVE" && f["target_displacement"] == 50);
  CHECK(r.tx.sent.back().id == kArbitrationId && r.tx.sent.back().dlc == 8);

  // 3) Same value for 100 more control cycles: NOTHING more is sent (no streaming).
  for (int i = 0; i < 100; ++i) r.write(0.04 + i * 0.001);
  CHECK(r.tx.sent.size() == 1);

  // 4) Value changes -> exactly one new frame
  r.C("vps_screw_rotation/position") = 0.080 / kVpsMult;
  r.write(0.2);
  CHECK(r.tx.sent.size() == 2);
  decode_cmd(r.tx.sent.back(), &name);
  CHECK(decode_cmd(r.tx.sent.back(), &name)["target_displacement"] == 80);

  // 5) Out-of-range (1200 mm > 1000 mm field maximum) is rejected, not clamped, not sent
  r.C("vps_screw_rotation/position") = 1.2 / kVpsMult;
  r.write(0.3);
  CHECK(r.tx.sent.size() == 2);
  CHECK((r.core->fault_flags(r.at(0.3)) & kFaultCommandRejected) != 0);
  CHECK(!r.core->take_events().empty());

  // 6) Wrong sign (positive rad = negative displacement) is also rejected
  r.C("vps_screw_rotation/position") = 10.0;
  r.write(0.31);
  CHECK(r.tx.sent.size() == 2);
}

static void test_signed_and_selectors() {
  std::printf("pump / led / signed fields\n");
  Rig r;
  std::string name;
  r.C("pump_1/power") = 0.5;
  r.C("pump_2/power") = -0.25;
  r.C("led_445/on") = 1.0;
  r.write(0.0);
  CHECK(r.tx.sent.size() == 3);
  std::map<std::string, std::map<std::string, int64_t>> by;
  std::vector<std::map<std::string, int64_t>> pumps;
  for (auto& fr : r.tx.sent) {
    auto d = decode_cmd(fr, &name);
    if (name == "URC_PUMP_RUN") pumps.push_back(d);
    if (name == "LED_SET") { CHECK(d["selector"] == 1 && d["on"] == 1); }
  }
  CHECK(pumps.size() == 2);
  for (auto& p : pumps) {
    if (p["selector"] == 0) CHECK(p["power"] == 10000);
    if (p["selector"] == 1) CHECK(p["power"] == -5000);              // two's complement round trip
  }
}

static void test_triggers() {
  std::printf("trigger commands (home / kill / timed)\n");
  Rig r;
  std::string name;
  r.C("vps_control/home") = 0.0;                                      // baseline, not a command
  r.write(0.0);
  CHECK(r.tx.sent.empty());
  r.C("vps_control/home") = 1.0;                                      // counter incremented -> fire once
  r.write(0.01); r.write(0.02); r.write(0.03);
  CHECK(r.tx.sent.size() == 1);
  auto f = decode_cmd(r.tx.sent.back(), &name);
  CHECK(name == "VPS_HOME" && f["home"] == 1);
  r.C("vps_control/home") = 2.0;                                      // again
  r.write(0.04);
  CHECK(r.tx.sent.size() == 2);

  r.C("payload_control/kill") = 1.0;
  r.write(0.05);
  CHECK(r.tx.sent.size() == 3);
  decode_cmd(r.tx.sent.back(), &name);
  CHECK(name == "KILL");

  // Timed action needs BOTH parameters; trigger waits for them without being lost.
  Rig e(kCompErc);
  e.C("spool/rotate_trigger") = 1.0;
  e.write(0.0);
  CHECK(e.tx.sent.empty());                                           // rpm / duration not set yet
  e.C("spool/rotate_rpm") = 600;
  e.C("spool/rotate_duration") = 2.5;                                 // seconds -> 25 deciseconds
  e.write(0.1);
  CHECK(e.tx.sent.size() == 1);
  auto g = decode_cmd(e.tx.sent.back(), &name);
  CHECK(name == "SPOOL_ROTATE" && g["speed_rpm"] == 600 && g["duration_ds"] == 25);
}

static void test_strict_competition() {
  std::printf("strict competition\n");
  Rig r(kCompUrc);                                                    // URC run
  r.C("erc_drill/rpm") = 300;                                         // an ERC command
  r.write(0.0);
  CHECK(r.tx.sent.empty());
  CHECK((r.core->fault_flags(r.at(0.0)) & kFaultCommandRejected) != 0);
}

static void test_retry_and_fault() {
  std::printf("readback, retry, fault\n");
  Rig r(kCompUrc, 2);
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 0.0);
  r.read(0.01);
  r.C("vps_screw_rotation/position") = 0.05 / kVpsMult;
  r.write(0.02);
  CHECK(r.tx.sent.size() == 1);

  // Payload never moves.  Before the 60 s timeout nothing is resent.
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 59.9);
  r.read(59.9); r.write(59.9);
  CHECK(r.tx.sent.size() == 1);
  // After the timeout: bounded resends (2), then a latched fault -- never a stream.
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 60.5); r.read(60.5); r.write(60.5);
  CHECK(r.tx.sent.size() == 2);
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 121.0); r.read(121.0); r.write(121.0);
  CHECK(r.tx.sent.size() == 3);
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 182.0); r.read(182.0); r.write(182.0);
  CHECK(r.tx.sent.size() == 3);
  CHECK((r.core->fault_flags(r.at(182.0)) & kFaultCommandNotReached) != 0);
  for (int i = 0; i < 50; ++i) { r.write(183.0 + i); }
  CHECK(r.tx.sent.size() == 3);                                       // fault does not cause more traffic

  // and the happy path: state arrives -> pending clears, no resend ever
  Rig g;
  g.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 0.0); g.read(0.01);
  g.C("vps_screw_rotation/position") = 0.05 / kVpsMult;
  g.write(0.02);
  g.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 50}}), 5.0); g.read(5.0); g.write(5.0);
  g.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 50}}), 100.0); g.read(100.0); g.write(100.0);
  CHECK(g.tx.sent.size() == 1);
  CHECK((g.core->fault_flags(g.at(100.0)) & kFaultCommandNotReached) == 0);

  // clear_faults counter clears latched bits
  r.C("payload_control/clear_faults") = 0.0; r.write(190.0);
  r.C("payload_control/clear_faults") = 1.0; r.write(191.0);
  r.rx(telem("TELEM_URC_GENERAL", {{"vps_displacement", 0}}), 191.0);
  CHECK((r.core->fault_flags(r.at(191.0)) & kFaultCommandNotReached) == 0);
}

static void test_tx_failure_and_safe_state() {
  std::printf("tx failure / safe state\n");
  Rig r;
  r.tx.fail = true;
  r.C("ultrasonic_mixer/on") = 1.0;
  r.write(0.0);
  CHECK((r.core->fault_flags(r.at(0.0)) & kFaultTxFailed) != 0);
  r.tx.fail = false;
  r.write(0.1);                                                       // failed send is retried next cycle
  CHECK(r.tx.sent.size() == 1);

  r.tx.sent.clear();
  r.core->send_safe_state(r.tx);
  CHECK(r.tx.sent.size() == 9);
  std::string name;
  int stops = 0;
  for (auto& fr : r.tx.sent) {
    auto d = decode_cmd(fr, &name);
    if (name == "URC_PUMP_RUN" || name == "ERC_PUMP_RUN") { CHECK(d["power"] == 0); ++stops; }
    if (name == "URC_DSS_DRILL_SET" || name == "ERC_DSS_DRILL_SET") { CHECK(d["rpm"] == 0); ++stops; }
    CHECK(name != "VPS_MOVE" && name != "URC_DSS_MOVE");            // never a position move
  }
  CHECK(stops == 5);
}

static void test_bind_errors() {
  std::printf("bind errors\n");
  Core core{CoreConfig{}};
  auto rep = core.bind({"vps_screw_rotation/position", "mystery/value"}, {"vps_screw_rotation/position"});
  bool saw_mystery = false;
  for (auto& e : rep.errors) if (e.find("mystery/value") != std::string::npos) saw_mystery = true;
  CHECK(saw_mystery);                                                 // an interface nobody can serve is caught at startup
}

int main(int argc, char** argv) {
  test_codec();
  test_vectors(argc > 1 ? argv[1] : "test/vectors.txt");
  test_table_sanity();
  test_telemetry();
  test_erc_mode();
  test_value_commands();
  test_signed_and_selectors();
  test_triggers();
  test_strict_competition();
  test_retry_and_fault();
  test_tx_failure_and_safe_state();
  test_bind_errors();
  std::printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
