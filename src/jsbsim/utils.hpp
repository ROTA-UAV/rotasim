#ifndef UTILS_H
#define UTILS_H

#include <godot_cpp/classes/project_settings.hpp>
#include <FGFDMExec.h>

using namespace godot;

namespace utils {
    SGPath jsb_path_from_gd_string(const String &p_path);
    Vector3 godot_vec_from_jsb_vec(const JSBSim::FGColumnVector3 &p_vec);
    JSBSim::FGColumnVector3 feet_to_meters_vec(const JSBSim::FGColumnVector3 &p_vec);
    double pps_to_hpa(double pps);
    double nanotesla_to_gauss(double nt);
    double wrap_pi_deg(double deg);
    double wrap_pi(double rad);
}

#endif