#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/string.hpp>

#include <FGFDMExec.h>

namespace godot {

class FDM {

public:
  FDM() = default;
  FDM(String p_model_name, String p_init_name, double p_frequency, uint32_t p_speed);

  JSBSim::FGFDMExec& get_fdm_exec();

  double get_frequency() const;
  void set_frequency(double p_frequency);

  String get_model_name() const;
  void set_model_name(const String &p_model_name);

  String get_init_name() const;
  void set_init_name(const String &p_init_name);

  double get_input_aileron() const;
  void set_input_aileron(double p_input_aileron);

  double get_input_elevator() const;
  void set_input_elevator(double p_input_elevator);

  double get_input_rudder() const;
  void set_input_rudder(double p_input_rudder);

  double get_input_throttle() const;
  void set_input_throttle(double p_input_throttle);

  double get_responsiveness() const;
  void set_responsiveness(double p_responsiveness);

  uint32_t get_num_of_iters() const;
  void set_num_of_iters(uint32_t p_num_of_iters);

  Vector3 get_rotation() const;
  Vector3 get_position() const;

  /* Runs the jsbsim one iteration */
  void step();
  /* Runs jsbsim multiple times according to the speed */
  void run();


private:
  JSBSim::FGFDMExec fdm_exec{};
  String model_name{};
  String init_name{};
  bool initialized{false};
  JSBSim::FGLocation initial_location{};

  double frequency{1000.0};
  uint32_t num_of_iters{1};

  double input_aileron{};
  double input_elevator{};
  double input_rudder{};
  double input_throttle{};

  Vector3 rotation{};
  Vector3 position{};

  void copy_from_jsbsim();
  void copy_to_jsbsim();
  JSBSim::FGLocation get_aircraft_location() const;
};

} // namespace godot