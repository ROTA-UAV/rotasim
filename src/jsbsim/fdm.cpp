#include "fdm.hpp"
#include "utils.hpp"

#include <FGFDMExec.h>
#include <FGJSBBase.h>
#include <initialization/FGInitialCondition.h>
#include <models/FGFCS.h>
#include <models/FGAuxiliary.h>

#include <cstdio>

#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

FDM::FDM(String p_model_name, String p_init_name, double p_frequency, uint32_t p_num_of_iters)
    : model_name(p_model_name), init_name(p_init_name), frequency(p_frequency), num_of_iters(p_num_of_iters) {
  UtilityFunctions::print("Loading model ", model_name, ".");

  // set custom path for JSBSim data
  fdm_exec.SetRootDir(utils::jsb_path_from_gd_string("res://jsbsim_data/aircraft"));
  fdm_exec.SetAircraftPath(SGPath(""));
  fdm_exec.SetEnginePath(SGPath("Engines"));
  fdm_exec.SetSystemsPath(SGPath("Systems"));
  fdm_exec.DisableOutput();

  bool result = fdm_exec.LoadModel(
    SGPath(""),
    SGPath("Engine"),
    SGPath("Systems"),
    model_name.utf8().get_data());
  if (!result) {
    UtilityFunctions::print("Failed to load model");
    return;
  }

  fdm_exec.Setdt(1.0 / frequency);
  fdm_exec.GetIC()->Load(utils::jsb_path_from_gd_string(init_name));
  fdm_exec.RunIC();

  // run the simulation for 10 seconds to stabilize it
  copy_to_jsbsim();
  for (uint64_t steps = 10.0 / fdm_exec.GetDeltaT(); steps > 0; --steps) {
    fdm_exec.Run();
  }

  initial_location = get_aircraft_location();
  copy_from_jsbsim();
  initialized = true;
}

void FDM::step() {
  copy_to_jsbsim();
  fdm_exec.Run();
  copy_from_jsbsim();
}

void FDM::run() {
  for (uint32_t i = 0; i < num_of_iters; ++i) {
    step();
  }
}

void FDM::copy_to_jsbsim() {
  fdm_exec.SetPropertyValue("fcs/aileron-cmd-norm", input_aileron);
  fdm_exec.SetPropertyValue("fcs/elevator-cmd-norm", input_elevator);
  fdm_exec.SetPropertyValue("fcs/rudder-cmd-norm", input_rudder);
  fdm_exec.SetPropertyValue("fcs/throttle-cmd-norm", input_throttle);

  // UtilityFunctions::print("Aileron: ", input_aileron);
  // UtilityFunctions::print("Elevator: ", input_elevator);
  // UtilityFunctions::print("Rudder: ", input_rudder);
  // UtilityFunctions::print("Throttle: ", input_throttle);
}

void FDM::copy_from_jsbsim() {
  double x = fdm_exec.GetPropagate()->GetEuler(2);
  double y = -fdm_exec.GetPropagate()->GetEuler(3);
  double z = -fdm_exec.GetPropagate()->GetEuler(1);
  rotation = Vector3(x, y, z);

  // use initial location as starting point
  JSBSim::FGColumnVector3 location_dt = initial_location.LocationToLocal(get_aircraft_location());
  position = utils::godot_vec_from_jsb_vec(utils::feet_to_meters_vec(location_dt));

  // UtilityFunctions::print(std::to_string(fdm_exec->GetSimTime()).c_str(), ",", std::to_string(fdm_exec->GetAuxiliary()->GetNz()).c_str());
}

JSBSim::FGFDMExec& FDM::get_fdm_exec() { return fdm_exec; }

JSBSim::FGLocation FDM::get_aircraft_location() const {
  return fdm_exec.GetPropagate()->GetLocation();
}

double FDM::get_frequency() const { return frequency; }
void FDM::set_frequency(double p_frequency) {
  frequency = p_frequency;
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

uint32_t FDM::get_num_of_iters() const { return num_of_iters; }
void FDM::set_num_of_iters(uint32_t p_num_of_iters) { num_of_iters = p_num_of_iters; }

Vector3 FDM::get_rotation() const { return rotation; }
Vector3 FDM::get_position() const { return position; }