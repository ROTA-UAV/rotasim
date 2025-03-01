#pragma once

#include <jsbsim/fdm.hpp>

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tcp_server.hpp>
#include <godot_cpp/classes/stream_peer_tcp.hpp>
#include <godot_cpp/classes/thread.hpp>

#include <FGFDMExec.h>
#include <mavsdk/mavlink_include.h>

#include <memory>

namespace godot {
class Link : public Node3D {
GDCLASS(Link, Node3D);

public:
    void _ready() override;
    void _exit_tree() override;

    void set_lockstep_enabled(bool lockstep);
    bool get_lockstep_enabled() const;

    void set_model_name(const String& p_model_name);
    String get_model_name() const;

    void set_init_name(const String& p_init_name);
    String get_init_name() const;

    void set_fdm_frequency(double p_frequency);
    double get_fdm_frequency() const;

    void set_loop_time(double p_loop_time);
    double get_loop_time() const;

    void set_speed(double p_speed);
    double get_speed() const;

protected:
    static void _bind_methods();

private:
    static constexpr const char* address{"*"};
    static constexpr uint8_t system_id{1};
    static constexpr uint8_t component_id{200};

    // px4_sitl looks for ports starting with this and increments it for every instance
    static constexpr uint16_t default_port{4560};
    static uint16_t port_counter;
    uint16_t port{};
    bool first_packet_received{false};
    bool sending{true};

    Ref<TCPServer> server{};
    Ref<StreamPeerTCP> autopilot{};
    bool is_connected{false};
    bool lockstep_enabled{false};
    int64_t tick_count{0};

    std::unique_ptr<FDM> fdm{};
    JSBSim::FGFDMExec* fdm_exec{};
    String model_name{};
    String init_name{};
    double loop_time{4.0};
    double fdm_frequency{1000.0};
    double speed{1.0};
    uint32_t num_of_fdm_iter{static_cast<uint32_t>(loop_time / (1000.0 / fdm_frequency) * speed)};

    Thread* thread{};

    void run();
    bool connect();
    void disconnect();
    bool check_connection();
    void handle_packages();
    bool send_state_msg() const;
    bool send_gps_msg() const;
    bool send_sensor_msg() const;
    bool every_ms(double interval_ms) const;
    void update_sim();
    void send_timesync();
    void send_heartbeat();
    bool recv_msg(mavlink_message_t &msg);
    static mavlink_message_t parse_msg(PackedByteArray data);
    bool handle_msg(const mavlink_message_t &msg);
    void handle_actuator_controls(const mavlink_message_t& msg);
};
}