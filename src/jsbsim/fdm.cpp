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

  // frequency
  ClassDB::bind_method(D_METHOD("get_frequency"), &FDM::get_frequency);
  ClassDB::bind_method(D_METHOD("set_frequency", "p_frequency"),
                       &FDM::set_frequency);
  ClassDB::add_property("FDM", PropertyInfo(Variant::FLOAT, "frequency"),
                        "set_frequency", "get_frequency");

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

FDM::FDM() : fdm_exec(nullptr), model_name(""), initialized(false), frequency(250.0),
             input_aileron(0.0), input_elevator(0.0), input_rudder(0.0), input_throttle(0.0) {}

void FDM::_ready() {
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
  Engine::get_singleton()->set_max_physics_steps_per_frame(INT32_MAX);
  Engine::get_singleton()->set_physics_ticks_per_second(frequency);
  fdm_exec->Setdt(1.0 / frequency);

  fdm_exec->GetIC()->Load(SGPath("initGrnd.xml"));
  fdm_exec->RunIC();

  copy_from_jsbsim();
  initial_location = get_aircraft_location();
  initialized = true;
}

FDM::~FDM() {}

void FDM::_physics_process(double delta) {
  if (Engine::get_singleton()->is_editor_hint()) {
    return;
  }

  if (!initialized) {
    return;
  }

  copy_to_jsbsim(delta);
  // UtilityFunctions::print("Input: ", fdm_exec->GetPropertyValue("fcs/throttle-input"));
  fdm_exec->Run();
  // UtilityFunctions::print("Output: ", fdm_exec->GetPropertyValue("fcs/throttle-cmd-norm"));
  copy_from_jsbsim();
}

void FDM::copy_to_jsbsim(double delta) {
  fdm_exec->GetFCS()->SetDaCmd(input_aileron);
  fdm_exec->GetFCS()->SetDeCmd(input_elevator);
  fdm_exec->GetFCS()->SetDrCmd(input_rudder);
  fdm_exec->SetPropertyValue("fcs/throttle-input", input_throttle);

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
}

std::shared_ptr<JSBSim::FGFDMExec> FDM::get_fdm_exec() const { return fdm_exec; }

JSBSim::FGLocation FDM::get_aircraft_location() const {
  return fdm_exec->GetPropagate()->GetLocation();
}

double FDM::get_frequency() const { return frequency; }
void FDM::set_frequency(double p_frequency) {
  frequency = p_frequency;
}

String FDM::get_model_name() const { return model_name; }
void FDM::set_model_name(const String &p_model_name) {
  model_name = p_model_name;
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