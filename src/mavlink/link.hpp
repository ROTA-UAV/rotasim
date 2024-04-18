#pragma once

#include <jsbsim/fdm.hpp>

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tcp_server.hpp>

#include <FGFDMExec.h>
#include <mavsdk/mavlink_include.h>

#include <memory>

namespace godot {
class Link : public Node {
GDCLASS(Link, Node);

public:
    Link();
    ~Link();

    void _physics_process(double delta) override;
    void _ready() override;
    void _exit_tree() override;

    void set_fdm_path(const NodePath& path);
    NodePath get_fdm_path() const;

protected:
    static void _bind_methods();

private:
    static constexpr const char* address = "*";
    static constexpr const uint16_t port = 4560; // px4 sitl connects on this port
    static constexpr uint8_t system_id = 1;
    static constexpr uint8_t component_id = 200;

    Ref<TCPServer> server;
    Ref<StreamPeerTCP> autopilot;

    NodePath fdm_path;
    std::shared_ptr<JSBSim::FGFDMExec> fdm;
    FDM* fdm_node;

    void send_data() const;
    void send_gps_data() const;
    void send_sensor_data() const;

    void recv_data();
    void handle_message(const mavlink_message_t& msg);
    void handle_actuator_controls(const mavlink_message_t& msg);
};
}