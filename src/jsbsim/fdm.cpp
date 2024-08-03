#include "fdm.hpp"
#include "utils.hpp"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>

#include <cstdio>

#include <gdextension_interface.h>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/global_constants.hpp>
#include <godot_cpp/core/property_info.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>

using namespace godot;

void FDM::_bind_methods() {
  // model name
  ClassDB::bind_method(D_METHOD("get_model_name"), &FDM::get_model_name);
  ClassDB::bind_method(D_METHOD("set_model_name", "p_model_name"),
                       &FDM::set_model_name);
  ClassDB::add_property("FDM", PropertyInfo(Variant::STRING, "model_name"),
                        "set_model_name", "get_model_name");

  // init name
  ClassDB::bind_method(D_METHOD("get_init_name"), &FDM::get_init_name);
  ClassDB::bind_method(D_METHOD("set_init_name", "p_init_name"),
                       &FDM::set_init_name);
  ClassDB::add_property("FDM", PropertyInfo(Variant::STRING, "init_name"),
                        "set_init_name", "get_init_name");

  // frequency
  ClassDB::bind_method(D_METHOD("get_frequency"), &FDM::get_frequency);
  ClassDB::bind_method(D_METHOD("set_frequency", "p_frequency"),
                       &FDM::set_frequency);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "frequency"),
                        "set_frequency", "get_frequency");

  // speed
  ClassDB::bind_method(D_METHOD("get_speed"), &FDM::get_speed);
  ClassDB::bind_method(D_METHOD("set_speed", "p_speed"),
                       &FDM::set_speed);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "speed"),
                        "set_speed", "get_speed");

  // aileron input
  ClassDB::bind_method(D_METHOD("get_input_aileron"), &FDM::get_input_aileron);
  ClassDB::bind_method(D_METHOD("set_input_aileron", "p_input_aileron"),
                       &FDM::set_input_aileron);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "aileron", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_aileron", "get_input_aileron");

  // elevator input
  ClassDB::bind_method(D_METHOD("get_input_elevator"), &FDM::get_input_elevator);
  ClassDB::bind_method(D_METHOD("set_input_elevator", "p_input_elevator"),
                       &FDM::set_input_elevator);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "elevator", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_elevator", "get_input_elevator");

  // rudder input
  ClassDB::bind_method(D_METHOD("get_input_rudder"), &FDM::get_input_rudder);
  ClassDB::bind_method(D_METHOD("set_input_rudder", "p_input_rudder"),
                       &FDM::set_input_rudder);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "rudder", PROPERTY_HINT_RANGE, "-1,1,0.01"),
                        "set_input_rudder", "get_input_rudder");

  // throttle input
  ClassDB::bind_method(D_METHOD("get_input_throttle"), &FDM::get_input_throttle);
  ClassDB::bind_method(D_METHOD("set_input_throttle", "p_input_throttle"),
                       &FDM::set_input_throttle);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "throttle", PROPERTY_HINT_RANGE, "0,1,0.01"),
                        "set_input_throttle", "get_input_throttle");
}

void FDM::_enter_tree() {
  if (Engine::get_singleton()->is_editor_hint()) {
    return;
  }

  if (model_name.is_empty()) {
    UtilityFunctions::print("Model name is empty.");
    return;
  }

  UtilityFunctions::print("Loading model ", model_name, ".");

  // set custom path for JSBSim data
  fdm_exec = std::make_shared<JSBSim::FGFDMExec>();
  fdm_exec->SetRootDir(utils::jsb_path_from_gd_string("res://jsbsim_data"));
  fdm_exec->SetAircraftPath(SGPath("aircraft"));
  fdm_exec->SetEnginePath(SGPath("engine"));
  fdm_exec->SetSystemsPath(SGPath("systems"));
  fdm_exec->DisableOutput();

  bool result = fdm_exec->LoadModel(
    SGPath("aircraft"),
    SGPath("engine"),
    SGPath("systems"),
    model_name.utf8().get_data());
  if (!result) {
    UtilityFunctions::print("Failed to load model");
    return;
  }

  // this node uses godot's _physics_process method to update the simulation
  // Engine::get_singleton()->set_max_physics_steps_per_frame(INT32_MAX);
  // Engine::get_singleton()->set_physics_ticks_per_second(frequency);
  fdm_exec->Setdt(1.0 / frequency);

  fdm_exec->GetIC()->Load(utils::jsb_path_from_gd_string(init_name));
  fdm_exec->RunIC();

  // run the simulation for 10 seconds to stabilize it
  // copy_to_jsbsim();
  // for (uint64_t steps = 10.0 / fdm_exec->GetDeltaT(); steps > 0; --steps) {
  //   fdm_exec->Run();
  // }
  copy_from_jsbsim();

  initial_location = get_aircraft_location();
  initialized = true;
}

void FDM::_physics_process(double delta) {
  if (Engine::get_singleton()->is_editor_hint()) {
    return;
  }

  if (!initialized) {
    return;
  }

  uint32_t ticks_per_second = Engine::get_singleton()->get_physics_ticks_per_second();
  // divide number of sim runs in a second by the number of ticks per second to get the number of runs per tick
  num_of_runs += (frequency / static_cast<double>(ticks_per_second)) * speed;

  for (;num_of_runs > 0.0; num_of_runs -= 1.0) {
    copy_to_jsbsim();
    fdm_exec->Run();
    copy_from_jsbsim();
  }
}

void FDM::copy_to_jsbsim() {
  fdm_exec->SetPropertyValue("fcs/aileron-cmd-norm", input_aileron);
  fdm_exec->SetPropertyValue("fcs/elevator-cmd-norm", input_elevator);
  fdm_exec->SetPropertyValue("fcs/rudder-cmd-norm", input_rudder);
  fdm_exec->SetPropertyValue("fcs/throttle-cmd-norm", input_throttle);

  // UtilityFunctions::print("Aileron: ", input_aileron);
  // UtilityFunctions::print("Elevator: ", input_elevator);
  // UtilityFunctions::print("Rudder: ", input_rudder);
  // UtilityFunctions::print("Throttle: ", input_throttle);
}

void FDM::copy_from_jsbsim() {
  double x = fdm_exec->GetPropagate()->GetEuler(2);
  double y = -fdm_exec->GetPropagate()->GetEuler(3);
  double z = -fdm_exec->GetPropagate()->GetEuler(1);
  set_rotation(Vector3(x, y, z));

  // use initial location as starting point
  JSBSim::FGColumnVector3 location_dt = initial_location.LocationToLocal(get_aircraft_location());
  set_position(utils::godot_vec_from_jsb_vec(utils::feet_to_meters_vec(location_dt)));

  // UtilityFunctions::print(std::to_string(fdm_exec->GetSimTime()).c_str(), ",", std::to_string(fdm_exec->GetAuxiliary()->GetNz()).c_str());
}

std::shared_ptr<JSBSim::FGFDMExec> FDM::get_fdm_exec() const { return fdm_exec; }

JSBSim::FGLocation FDM::get_aircraft_location() const {
  return fdm_exec->GetPropagate()->GetLocation();
}

double FDM::get_frequency() const { return frequency; }
void FDM::set_frequency(double p_frequency) {
  frequency = p_frequency;
}

double FDM::get_speed() const { return speed; }
void FDM::set_speed(double p_speed) {
  speed = p_speed;
}

String FDM::get_model_name() const { return model_name; }
void FDM::set_model_name(const String &p_model_name) {
  model_name = p_model_name;
}

String FDM::get_init_name() const { return init_name; }
void FDM::set_init_name(const String &p_init_name) {
  init_name = p_init_name;
}

double FDM::get_input_aileron() const { return input_aileron; }
void FDM::set_input_aileron(double p_input_aileron) {
  input_aileron = p_input_aileron;
}

double FDM::get_input_elevator() const { return input_elevator; }
void FDM::set_input_elevator(double p_input_elevator) {
  input_elevator = p_input_elevator;
}

double FDM::get_input_rudder() const { return input_rudder; }
void FDM::set_input_rudder(double p_input_rudder) {
  input_rudder = p_input_rudder;
}

double FDM::get_input_throttle() const { return input_throttle; }
void FDM::set_input_throttle(double p_input_throttle) {
  input_throttle = p_input_throttle;
}