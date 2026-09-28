/// @file
#pragma once

#include <option/has_nozzle_pid_autotune.h>
static_assert(HAS_NOZZLE_PID_AUTOTUNE());

#include <config_store/store_instance.hpp>
#include <module/temperature/hotend_regulator/hotend_regulator.hpp>

/// Nozzle PID terms persisted in the config store (set by M301 and M303 U1)
namespace nozzle_pid_store {

/// @returns base with the saved Kp, Ki and Kd applied
inline HotendPIDConfig load(HotendPIDConfig base = {}) {
    base.Kp = config_store().nozzle_pid_kp.get();
    base.Ki = scalePID_i(config_store().nozzle_pid_ki.get());
    base.Kd = scalePID_d(config_store().nozzle_pid_kd.get());
    return base;
}

/// Saves Kp, Ki and Kd of pid
inline void save(const HotendPIDConfig &pid) {
    config_store().nozzle_pid_kp.set(pid.Kp);
    config_store().nozzle_pid_ki.set(unscalePID_i(pid.Ki));
    config_store().nozzle_pid_kd.set(unscalePID_d(pid.Kd));
}

} // namespace nozzle_pid_store
