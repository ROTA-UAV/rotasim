#pragma once

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/string.hpp>

#include <FGFDMExec.h>

namespace godot {

class FDM : public Node3D {
  GDCLASS(FDM, Node3D)

public:
  FDM() = default;

  std::shared_ptr<JSBSim::FGFDMExec> get_fdm_exec() const;

  void _enter_tree() override;
  void _physics_process(double delta) override;

  double get_frequency() const;
  void set_frequency(double p_frequency);

  double get_speed() const;
  void set_speed(double p_speed);

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

private:
  std::shared_ptr<JSBSim::FGFDMExec> fdm_exec{};
  String model_name{};
  String init_name{};
  bool initialized{false};
  JSBSim::FGLocation initial_location{};

  double frequency{1000.0};
  double speed{1.0};
  double num_of_runs{};

  double input_aileron{};
  double input_elevator{};
  double input_rudder{};
  double input_throttle{};

  void copy_from_jsbsim();
  void copy_to_jsbsim();
  JSBSim::FGLocation get_aircraft_location() const;

protected:
  static void _bind_methods();
};

} // namespace godot