#ifndef FDM_H
#define FDM_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/string.hpp>

#include <FGFDMExec.h>

namespace godot {

class FDM : public Node3D {
  GDCLASS(FDM, Node3D)

public:
  FDM();
  ~FDM();

  std::shared_ptr<JSBSim::FGFDMExec> get_fdm_exec() const;

  void _physics_process(double delta) override;
  void _enter_tree() override;

  double get_frequency() const;
  void set_frequency(double p_frequency);

  String get_model_name() const;
  void set_model_name(const String &p_model_name);

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
  std::shared_ptr<JSBSim::FGFDMExec> fdm_exec;
  String model_name;
  bool initialized;
  JSBSim::FGLocation initial_location;

  double frequency;

  double input_aileron;
  double input_elevator;
  double input_rudder;
  double input_throttle;

  void copy_from_jsbsim();
  void copy_to_jsbsim(double delta);
  JSBSim::FGLocation get_aircraft_location() const;

protected:
  static void _bind_methods();
};

} // namespace godot

#endif
