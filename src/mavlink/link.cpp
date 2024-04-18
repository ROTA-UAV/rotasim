#include "link.hpp"

#include <jsbsim/fdm.hpp>
#include <jsbsim/utils.hpp>

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/tcp_server.hpp>
#include <godot_cpp/classes/stream_peer_tcp.hpp>

#include <mavsdk/mavlink_include.h>
#include <models/FGAccelerations.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>

#include <memory>

using namespace godot;

void Link::_bind_methods() {
    // fdm path
    ClassDB::bind_method(D_METHOD("get_fdm_path"), &Link::get_fdm_path);
    ClassDB::bind_method(D_METHOD("set_fdm_path", "path"), &Link::set_fdm_path);
    ClassDB::add_property("Link", PropertyInfo(Variant::NODE_PATH, "fdm_node"),
                          "set_fdm_path", "get_fdm_path");
}

Link::Link() : fdm_path(""), fdm(nullptr) {}

Link::~Link() {}

void Link::_ready() {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    // get reference to jsbsim fdm
    Node* node = get_node_or_null(fdm_path); // get_node didn't work
    if (node == nullptr) {
        UtilityFunctions::printerr("FDM node not found.");
        set_process_mode(Node::PROCESS_MODE_DISABLED);
        return;
    }
    fdm_node = Object::cast_to<FDM>(node);
    fdm = fdm_node->get_fdm_exec();

    // create server
    server.instantiate();
    server->listen(port, address);
}

void Link::_exit_tree() {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    // destroy server
    server->stop();
    server.unref();
}

void Link::_physics_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    if (autopilot.is_null()) {
        if (server->is_connection_available()) {
            autopilot = server->take_connection();
            autopilot->set_no_delay(true);
            UtilityFunctions::print("Connected to autopilot.");
        }
    } else {
        if (autopilot->get_status() == StreamPeerTCP::STATUS_ERROR ||
            autopilot->get_status() == StreamPeerTCP::STATUS_NONE)
        {
            autopilot->disconnect_from_host();
            autopilot.unref();
            UtilityFunctions::print("Disconnected from autopilot.");
            return;
        }
    }

    if (autopilot.is_valid() &&
        autopilot->poll() == Error::OK &&
        autopilot->get_status() == StreamPeerTCP::STATUS_CONNECTED)
    {
          send_data();
          if (fdm_node) {
            recv_data();
          }
    }
}

void Link::send_data() const {
    send_gps_data();
    send_sensor_data();
}

void Link::send_gps_data() const {
    const auto propagate = fdm->GetPropagate();
    mavlink_message_t msg;
    mavlink_hil_gps_t state;
    state.time_usec = fdm->GetSimTime() * 1e6;
    state.lat = propagate->GetLatitudeDeg() * 1e7;
    state.lon = propagate->GetLongitudeDeg() * 1e7;
    state.alt = propagate->GetAltitudeASLmeters() * 1e3;
    state.eph = fdm->GetPropertyValue("px4/gps-eph") * 1e2;
    state.epv = fdm->GetPropertyValue("px4/gps-epv") * 1e2;
    state.vel = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/gps-velocity")) * 1e2;
    state.vn = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/gps-v-north")) * 1e2;
    state.ve = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/gps-v-east")) * 1e2;
    state.vd = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/gps-v-down")) * 1e2;
    state.cog = utils::wrap_pi_deg(atan2(state.ve, state.vn) * (180.0 / M_PI)) * 1e2;
    state.fix_type = GPS_FIX_TYPE_3D_FIX;
    state.satellites_visible = 16; // UINT8_MAX gives error
    state.id = 0;
    state.yaw = propagate->GetEulerDeg(JSBSim::FGJSBBase::ePsi) * 1e2;

    mavlink_msg_hil_gps_encode_chan(system_id, component_id, MAVLINK_COMM_0, &msg, &state);
    PackedByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    mavlink_msg_to_send_buffer(buffer.ptrw(), &msg);

    Error result = autopilot->put_data(buffer);
    if (result != Error::OK) {
        UtilityFunctions::printerr("Failed to send GPS data to autopilot.");
    }
}

void Link::send_sensor_data() const {
    const auto propagate = fdm->GetPropagate();
    const auto accels = fdm->GetAccelerations();
    const auto atmo = fdm->GetAtmosphere();
    const auto auxilary = fdm->GetAuxiliary();
    const auto rng = fdm->GetRandomGenerator();

    mavlink_message_t msg;
    mavlink_hil_sensor_t state;
    state.time_usec = static_cast<uint64_t>(fdm->GetSimTime() * 1e6);
    state.xacc = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/acc-x"));
    state.yacc = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/acc-y"));
    state.zacc = JSBSim::FGJSBBase::FeetToMeters(fdm->GetPropertyValue("px4/acc-z"));
    state.xgyro = fdm->GetPropertyValue("px4/gyro-x");
    state.ygyro = fdm->GetPropertyValue("px4/gyro-y");
    state.zgyro = fdm->GetPropertyValue("px4/gyro-z");
    state.xmag = utils::nanotesla_to_gauss(fdm->GetPropertyValue("px4/mag-x"));
    state.ymag = utils::nanotesla_to_gauss(fdm->GetPropertyValue("px4/mag-y"));
    state.zmag = utils::nanotesla_to_gauss(fdm->GetPropertyValue("px4/mag-z"));
    state.abs_pressure = utils::pps_to_hpa(atmo->GetPressure()) + (rng->GetNormalRandomNumber() * 1e-6);
    state.diff_pressure = utils::pps_to_hpa(auxilary->Getqbar()) + (rng->GetNormalRandomNumber() * 1e-6);
    state.pressure_alt = JSBSim::FGJSBBase::FeetToMeters(atmo->GetPressureAltitude());
    state.temperature = JSBSim::FGJSBBase::RankineToCelsius(atmo->GetTemperature()) * rng->GetNormalRandomNumber() * 1e-6;
    state.fields_updated = HIL_SENSOR_UPDATED_XACC | HIL_SENSOR_UPDATED_YACC | HIL_SENSOR_UPDATED_ZACC |
                           HIL_SENSOR_UPDATED_XGYRO | HIL_SENSOR_UPDATED_YGYRO | HIL_SENSOR_UPDATED_ZGYRO |
                           HIL_SENSOR_UPDATED_XMAG | HIL_SENSOR_UPDATED_YMAG | HIL_SENSOR_UPDATED_ZMAG |
                           HIL_SENSOR_UPDATED_ABS_PRESSURE | HIL_SENSOR_UPDATED_DIFF_PRESSURE | HIL_SENSOR_UPDATED_PRESSURE_ALT |
                           HIL_SENSOR_UPDATED_TEMPERATURE;
    state.id = 0;

    mavlink_msg_hil_sensor_encode_chan(system_id, component_id, MAVLINK_COMM_0, &msg, &state);
    PackedByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    mavlink_msg_to_send_buffer(buffer.ptrw(), &msg);

    Error result = autopilot->put_data(buffer);
    if (result != Error::OK) {
        UtilityFunctions::printerr("Failed to send sensor data to autopilot.");
    }
}

void Link::recv_data() {
    int32_t available_bytes = autopilot->get_available_bytes();
    if (available_bytes <= 0) return;

    Array err_and_data = autopilot->get_data(available_bytes);
    int err(err_and_data[0]);
    if (err != Error::OK) {
        UtilityFunctions::printerr("Failed to receive data from autopilot.");
        return;
    }

    PackedByteArray data(err_and_data[1]);
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int64_t i = 0; i < data.size(); ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            handle_message(msg);
        }
    }


}

void Link::handle_message(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: {
            handle_actuator_controls(msg);
            break;
        }
        default: {
            // UtilityFunctions::print("Unknown message ID: ", msg.msgid);
            break;
        }
    }
}

void Link::handle_actuator_controls(const mavlink_message_t& msg) {
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

    // for (int i = 0; i < 16; ++i) {
    //     UtilityFunctions::print("Control ", i, ": ", controls.controls[i]);
    // }

    // any way to now which channel is which?
    double rudder = controls.controls[2];
    double motor = controls.controls[4];
    double left_aileron = controls.controls[5];
    double right_aileron = controls.controls[6];
    double elevator = -controls.controls[7];

    fdm_node->set_input_throttle(motor);
    fdm_node->set_input_aileron(right_aileron);
    fdm_node->set_input_elevator(elevator);
    fdm_node->set_input_rudder(rudder);

    // UtilityFunctions::print("Rudder: ", rudder);
    // UtilityFunctions::print("Motor: ", motor);
    // UtilityFunctions::print("Left Aileron: ", left_aileron);
    // UtilityFunctions::print("Right Aileron: ", right_aileron);
    // UtilityFunctions::print("Elevator: ", elevator);
}

void Link::set_fdm_path(const NodePath& path) { fdm_path = path; }
NodePath Link::get_fdm_path() const { return fdm_path; }