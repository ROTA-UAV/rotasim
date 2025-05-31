#include "link.hpp"

#include <chrono>
#include <thread>
#include <jsbsim/fdm.hpp>
#include <jsbsim/utils.hpp>

#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/tcp_server.hpp>
#include <godot_cpp/classes/stream_peer_tcp.hpp>
#include <godot_cpp/classes/thread.hpp>

#include <mavsdk/mavlink_include.h>
#include <models/FGAccelerations.h>
#include <models/FGAtmosphere.h>
#include <models/FGAuxiliary.h>

#include <memory>

using namespace godot;
using namespace std::chrono_literals;

void Link::_bind_methods() {
    // lockstep
    ClassDB::bind_method(D_METHOD("get_lockstep_enabled"), &Link::get_lockstep_enabled);
    ClassDB::bind_method(D_METHOD("set_lockstep_enabled", "lockstep"), &Link::set_lockstep_enabled);
    ClassDB::add_property("Link", PropertyInfo(Variant::BOOL, "lockstep_enabled"), "set_lockstep_enabled", "get_lockstep_enabled");

    // thread
    ClassDB::bind_method(D_METHOD("run"), &Link::run);

    // model name
    ClassDB::bind_method(D_METHOD("get_model_name"), &Link::get_model_name);
    ClassDB::bind_method(D_METHOD("set_model_name", "p_model_name"), &Link::set_model_name);
    ClassDB::add_property("Link", PropertyInfo(Variant::STRING, "model_name"), "set_model_name", "get_model_name");

    // init name
    ClassDB::bind_method(D_METHOD("get_init_name"), &Link::get_init_name);
    ClassDB::bind_method(D_METHOD("set_init_name", "p_init_name"), &Link::set_init_name);
    ClassDB::add_property("Link", PropertyInfo(Variant::STRING, "init_name"), "set_init_name", "get_init_name");

    // fdm time
    ClassDB::bind_method(D_METHOD("get_fdm_time"), &Link::get_fdm_time);
    ClassDB::bind_method(D_METHOD("set_fdm_time", "p_fdm_time"), &Link::set_fdm_time);
    ClassDB::add_property("Link", PropertyInfo(Variant::FLOAT, "fdm_time"), "set_fdm_time", "get_fdm_time");

    // loop time
    ClassDB::bind_method(D_METHOD("get_loop_time"), &Link::get_loop_time);
    ClassDB::bind_method(D_METHOD("set_loop_time", "p_loop_time"), &Link::set_loop_time);
    ClassDB::add_property("Link", PropertyInfo(Variant::FLOAT, "loop_time"), "set_loop_time", "get_loop_time");

    // speed
    ClassDB::bind_method(D_METHOD("get_speed"), &Link::get_speed);
    ClassDB::bind_method(D_METHOD("set_speed", "p_speed"), &Link::set_speed);
    ClassDB::add_property("Link", PropertyInfo(Variant::FLOAT, "speed"), "set_speed", "get_speed");
}

uint16_t Link::port_counter = Link::default_port;

void Link::_ready() {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    // create server
    port = port_counter++;
    server.instantiate();
    server->listen(port, address);

    thread = memnew(Thread);
    thread->start(Callable(this, "run"), Thread::PRIORITY_HIGH);
}

void Link::run() {
    UtilityFunctions::print("Thread started.");

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

        if (autopilot.is_null()) {
            if (server->is_connection_available()) {
                if (!connect()) {
                    UtilityFunctions::print("Failed to connect to autopilot.");
                    return;
                }
            }
        }

        if (is_connected) {
            if (!check_connection()) {
                UtilityFunctions::print("Connection lost.");
                return;
            }

            handle_packages();

            if (first_packet_received) {
                if (every_ms(25.0)) {
                    send_gps_msg();
                }

                if (every_ms(4000.0)) {
                    send_timesync();
                }

                if (every_ms(1000.0)) {
                    send_heartbeat();
                }
            }
        }

        // wait for the rest of the time
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        auto sleep_time = std::chrono::milliseconds(static_cast<int64_t>(loop_time)) - elapsed;
        this_thread::sleep_for(sleep_time);
        ++tick_count;
    }
}

bool Link::every_ms(double interval_ms) const {
    return tick_count % static_cast<int64_t>(interval_ms / static_cast<double>(loop_time)) == 0;
}

void Link::_exit_tree() {
    if (Engine::get_singleton()->is_editor_hint()) {
        return;
    }

    // destroy server
    server->stop();
    server.unref();
}

void Link::disconnect() {
    autopilot->disconnect_from_host();
    autopilot.unref();
    first_packet_received = false;
    sending = true;
    is_connected = false;
    tick_count = 0;
    fdm = nullptr;
    fdm_exec = nullptr;
}

bool Link::connect() {
    autopilot = server->take_connection();
    autopilot->set_no_delay(true);

    if (model_name.is_empty()) {
        UtilityFunctions::printerr("Model name is empty.");
        return false;
    }

    if (init_name.is_empty()) {
        UtilityFunctions::printerr("Init name is empty.");
        return false;
    }

    fdm = std::make_unique<FDM>(model_name, init_name, 1000.0 / fdm_time, num_of_fdm_iter);
    fdm_exec = &fdm->get_fdm_exec();

    {
        mavlink_message_t msg;
        if (!recv_msg(msg)) return false; 
        if (msg.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
            UtilityFunctions::printerr("Wrong message id:", msg.msgid);
            return false;
        }

        send_state_msg();
    }

    is_connected = true;
    start = std::chrono::high_resolution_clock::now();
    return true;
}

void Link::send_timesync() {
    mavlink_message_t msg;
    mavlink_timesync_t timesync;
    timesync.tc1 = 0;
    timesync.ts1 = static_cast<uint64_t>(fdm_exec->GetSimTime() * 1e6);

    mavlink_msg_timesync_encode_chan(system_id, component_id, MAVLINK_COMM_0, &msg, &timesync);
    PackedByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    mavlink_msg_to_send_buffer(buffer.ptrw(), &msg);

    Error result = autopilot->put_data(buffer);
    if (result != Error::OK) {
        UtilityFunctions::printerr("Failed to send TIMESYNC to autopilot.");
    }
}

void Link::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = MAV_TYPE_FIXED_WING;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    heartbeat.custom_mode = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    mavlink_msg_heartbeat_encode_chan(system_id, component_id, MAVLINK_COMM_0, &msg, &heartbeat);
    PackedByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    mavlink_msg_to_send_buffer(buffer.ptrw(), &msg);

    Error result = autopilot->put_data(buffer);
    if (result != Error::OK) {
        UtilityFunctions::printerr("Failed to send HEARTBEAT to autopilot.");
    }
}

void Link::update_sim() {
    fdm->run();

    Vector3 position = fdm->get_position();
    Vector3 rotation = fdm->get_rotation();

    call_deferred("set_position", position);
    call_deferred("set_rotation", rotation);
}

void Link::handle_packages() {
    if (!lockstep_enabled) {
        update_sim();
        send_sensor_msg();
        send_gps_msg();
        send_state_msg();
        mavlink_message_t msg;
        recv_msg(msg);
        handle_msg(msg);
        return;
    }

    if(!first_packet_received) {
        mavlink_message_t msg;
        if (recv_msg(msg)) {
            handle_msg(msg);
            first_packet_received = true;
            return;
        }
        else {
            update_sim();
            send_sensor_msg();
        }
    }
    else {
        if (sending) {
            update_sim();
            send_sensor_msg();
            sending = false;
        }
        else {
            mavlink_message_t msg;
            if (recv_msg(msg)) {
                handle_msg(msg);
                sending = true;
            }
        }
    }
}

bool Link::check_connection() {
    if (autopilot.is_null()) {
        return false;
    }
    autopilot->poll();
    if (autopilot->get_status() == StreamPeerTCP::STATUS_ERROR ||
        autopilot->get_status() == StreamPeerTCP::STATUS_NONE)
    {
        disconnect();
        UtilityFunctions::print("Disconnected from autopilot.");
        return false;
    }

    if (autopilot->get_status() == StreamPeerTCP::STATUS_CONNECTED)
    {
        return true;
    }
}

bool Link::send_state_msg() const {
    mavlink_message_t msg;
    mavlink_hil_state_t state;
    state.time_usec = static_cast<uint64_t>(fdm_exec->GetSimTime() * 1e6);
    state.roll = fdm_exec->GetPropertyValue("attitude/roll-deg") * 1e2;
    state.pitch = fdm_exec->GetPropertyValue("attitude/pitch-deg") * 1e2;
    state.yaw = fdm_exec->GetPropertyValue("attitude/yaw-deg") * 1e2;
    state.rollspeed = fdm_exec->GetPropertyValue("attitude/roll-rate-degps") * 1e2;
    state.pitchspeed = fdm_exec->GetPropertyValue("attitude/pitch-rate-degps") * 1e2;
    state.yawspeed = fdm_exec->GetPropertyValue("attitude/yaw-rate-degps") * 1e2;
    state.lat = fdm_exec->GetPropagate()->GetLatitudeDeg() * 1e7;
    state.lon = fdm_exec->GetPropagate()->GetLongitudeDeg() * 1e7;
    state.alt = fdm_exec->GetPropagate()->GetAltitudeASLmeters() * 1e3;
    state.vx = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("velocities/u-fps")) * 1e2;
    state.vy = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("velocities/v-fps")) * 1e2;
    state.vz = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("velocities/w-fps")) * 1e2;
    state.xacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("accelerations/pilot-x-fps_sec")) * 1e2;
    state.yacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("accelerations/pilot-y-fps_sec")) * 1e2;
    state.zacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("accelerations/pilot-z-fps_sec")) * 1e2;

    mavlink_msg_hil_state_encode_chan(system_id, component_id, MAVLINK_COMM_0, &msg, &state);
    PackedByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    mavlink_msg_to_send_buffer(buffer.ptrw(), &msg);
 
    Error result = autopilot->put_data(buffer);
    if (result != Error::OK) {
        UtilityFunctions::printerr("Failed to send STATE data to autopilot.");
        return false;
    }
    return true;
}

bool Link::send_gps_msg() const {
    const auto propagate = fdm_exec->GetPropagate();
    mavlink_message_t msg;
    mavlink_hil_gps_t state;
    state.time_usec = static_cast<uint64_t>(fdm_exec->GetSimTime() * 1e6);
    state.lat = propagate->GetLatitudeDeg() * 1e7;
    state.lon = propagate->GetLongitudeDeg() * 1e7;
    state.alt = propagate->GetAltitudeASLmeters() * 1e3;
    state.eph = fdm_exec->GetPropertyValue("px4/gps-eph") * 1e2;
    state.epv = fdm_exec->GetPropertyValue("px4/gps-epv") * 1e2;
    state.vel = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/gps-velocity")) * 1e2;
    state.vn = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/gps-v-north")) * 1e2;
    state.ve = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/gps-v-east")) * 1e2;
    state.vd = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/gps-v-down")) * 1e2;
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
        return false;
    }
    return true;
}

bool Link::send_sensor_msg() const {
    // UtilityFunctions::print("Sending sensor data.");
    const auto propagate = fdm_exec->GetPropagate();
    const auto accels = fdm_exec->GetAccelerations();
    const auto atmo = fdm_exec->GetAtmosphere();
    const auto auxilary = fdm_exec->GetAuxiliary();
    const auto rng = fdm_exec->GetRandomGenerator();

    mavlink_message_t msg;
    mavlink_hil_sensor_t state;
    state.time_usec = static_cast<uint64_t>(fdm_exec->GetSimTime() * 1e6);
    state.xacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/acc-x"));
    state.yacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/acc-y"));
    state.zacc = JSBSim::FGJSBBase::FeetToMeters(fdm_exec->GetPropertyValue("px4/acc-z"));
    state.xgyro = fdm_exec->GetPropertyValue("px4/gyro-x");
    state.ygyro = fdm_exec->GetPropertyValue("px4/gyro-y");
    state.zgyro = fdm_exec->GetPropertyValue("px4/gyro-z");
    state.xmag = utils::nanotesla_to_gauss(fdm_exec->GetPropertyValue("px4/mag-x"));
    state.ymag = utils::nanotesla_to_gauss(fdm_exec->GetPropertyValue("px4/mag-y"));
    state.zmag = utils::nanotesla_to_gauss(fdm_exec->GetPropertyValue("px4/mag-z"));
    state.abs_pressure = utils::pps_to_hpa(atmo->GetPressure()) + (rng->GetNormalRandomNumber() * 1e-6);
    state.diff_pressure = utils::pps_to_hpa(auxilary->Getqbar()) + (rng->GetNormalRandomNumber() * 1e-6);
    state.pressure_alt = JSBSim::FGJSBBase::FeetToMeters(atmo->GetPressureAltitude());
    state.temperature = JSBSim::FGJSBBase::RankineToCelsius(atmo->GetTemperature()) + (rng->GetNormalRandomNumber() * 1e-6);
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
        return false;
    }
    return true;
}

mavlink_message_t Link::parse_msg(PackedByteArray data) {
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int64_t i = 0; i < data.size(); ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
            // UtilityFunctions::print("Received message with ID: ", msg.msgid);
            return msg;
        }
    }
}

bool Link::recv_msg(mavlink_message_t &msg) {
    int32_t available_bytes = autopilot->get_available_bytes();
    if (available_bytes <= 0) {
        return false;
    }

    Array err_and_data = autopilot->get_data(available_bytes);
    int err(err_and_data[0]);
    if (err != Error::OK) {
        UtilityFunctions::printerr("Failed to receive data from autopilot.");
        return false;
    }

    PackedByteArray data(err_and_data[1]);
    msg = parse_msg(data);
    return true;
}

bool Link::handle_msg(const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS: {
            handle_actuator_controls(msg);
            return true;
        }
        default: {
            // UtilityFunctions::print("Unknown message ID: ", msg.msgid);
            return false;
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

    fdm->set_input_throttle(motor);
    fdm->set_input_aileron(right_aileron);
    fdm->set_input_elevator(elevator);
    fdm->set_input_rudder(rudder);

    // UtilityFunctions::print("Rudder: ", rudder);
    // UtilityFunctions::print("Motor: ", motor);
    // UtilityFunctions::print("Left Aileron: ", left_aileron);
    // UtilityFunctions::print("Right Aileron: ", right_aileron);
    // UtilityFunctions::print("Elevator: ", elevator);
}

void Link::set_lockstep_enabled(bool lockstep) { lockstep_enabled = lockstep; }
bool Link::get_lockstep_enabled() const { return lockstep_enabled; }

void Link::set_model_name(const String &p_model_name) { model_name = p_model_name; }
String Link::get_model_name() const { return model_name; }

void Link::set_init_name(const String &p_init_name) { init_name = p_init_name; }
String Link::get_init_name() const { return init_name; }

void Link::set_fdm_time(double p_fdm_time) {
    fdm_time = p_fdm_time;
    num_of_fdm_iter = loop_time / fdm_time * speed;
}
double Link::get_fdm_time() const { return fdm_time; }

void Link::set_loop_time(double p_loop_time) {
    loop_time = p_loop_time;
    num_of_fdm_iter = loop_time / fdm_time * speed;
}
double Link::get_loop_time() const { return loop_time; }

void Link::set_speed(double p_speed) {
    speed = p_speed;
    num_of_fdm_iter = loop_time / fdm_time * speed;
}
double Link::get_speed() const { return speed; }