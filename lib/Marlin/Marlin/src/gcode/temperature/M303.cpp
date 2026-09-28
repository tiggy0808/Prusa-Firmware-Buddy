/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if HAS_PID_HEATING

    #include "../gcode.h"

    #include <option/has_nozzle_pid_autotune.h>
    #if HAS_NOZZLE_PID_AUTOTUNE()
        #include <cmath>
        #include <cstdio>
        #include <cstdlib>
        #include <cstring>
        #include <expected>
        #include <memory>
        #include <numbers>

        #include "../../Marlin.h"
        #include "../../module/planner.h"
        #include "../../module/temperature.h"
        #include <gcode/gcode_parser.hpp>
        #include <feature/safety_timer/safety_timer.hpp>
        #include <feature/print_status_message/print_status_message_guard.hpp>
        #include <feature/print_status_message/print_status_message_mgr.hpp>
        #include <nozzle_pid_store.hpp>
        #include <raii/scope_guard.hpp>
    #endif

    #if HAS_NOZZLE_PID_AUTOTUNE()
namespace {

constexpr int16_t default_target_temp = 200;
constexpr int16_t min_target_temp = 100;

constexpr int default_cycles = 5;
constexpr int min_cycles = 3;
constexpr int max_cycles = 20;

/// Minimum time the heater stays on or off, so that sensor noise around the target doesn't flip it
constexpr millis_t min_relay_phase_ms = 5'000;

/// Give up when the temperature doesn't cross the target for this long
constexpr millis_t max_relay_phase_ms = 10 * 60'000;

/// Give up when the temperature overshoots the target by more than this
constexpr float max_overshoot = 20;

/// Give up this far below the hotend's maximum temperature, before MAXTEMP trips
constexpr float maxtemp_margin = 5;

constexpr uint32_t result_message_duration_ms = 60'000;

PrintStatusMessage::TypeRecordOf<PrintStatusMessage::custom>::Data make_message(const char *text) {
    return { std::shared_ptr<const char[]>(strdup(text), [](char *str) { free(str); }) };
}

/// Relay autotune as in upstream Marlin: the heater alternates between bias + d and bias - d
/// around the target, and the PID terms follow from the period and amplitude of the oscillation.
/// The heater output is forced through the hotend, so all of its thermal protections stay active.
std::expected<HotendPIDConfig, const char *> autotune_nozzle(PhysicalToolIndex tool, int16_t target, int ncycles, PrintStatusMessageGuard &status) {
    Hotend &hotend = Hotend::for_tool(tool);

    constexpr long max_pow = PID_MAX;
    long bias = max_pow / 2;
    long d = max_pow / 2;
    const auto set_heater = [&](long pwm) {
        return hotend.set_nozzle_heater_pwm_override(PWM255(static_cast<uint8_t>(std::clamp<long>(pwm, 0, max_pow))));
    };

    thermalManager.setTargetHotend(target, tool);
    if (!set_heater(bias + d)) {
        return std::unexpected("not supported by this hotend");
    }

    const float abort_temp = std::min<float>(target + max_overshoot, hotend.max_nozzle_temp() - maxtemp_margin);

    HotendPIDConfig tuned = hotend.nozzle_pid_config();
    bool has_result = false;

    bool heating = true;
    millis_t t1 = millis();
    millis_t t2 = t1;
    long t_high = 0;
    long t_low = 0;
    float max_t = 0;
    float min_t = 10'000;
    int cycles = 0;

    wait_for_heatup = true;
    while (true) {
        if (planner.draining() || !wait_for_heatup) {
            return std::unexpected("aborted");
        }

        idle(true);

        // Anything else changing the target (M104, cooldown, abort) ends the autotune
        if (hotend.nozzle_target_temp() != target) {
            return std::unexpected("aborted");
        }

        const auto maybe_temp = hotend.nozzle_temp();
        if (!maybe_temp.has_value()) {
            continue;
        }
        const float current = maybe_temp.value();
        max_t = std::max(max_t, current);
        min_t = std::min(min_t, current);

        const millis_t now = millis();

        if (heating && current > target && ELAPSED(now, t2 + min_relay_phase_ms)) {
            heating = false;
            set_heater(bias - d);
            t1 = now;
            t_high = t1 - t2;
            max_t = target;
        }

        if (!heating && current < target && ELAPSED(now, t1 + min_relay_phase_ms)) {
            heating = true;
            t2 = now;
            t_low = t2 - t1;

            if (cycles > 0) {
                bias += (d * (t_high - t_low)) / (t_low + t_high);
                bias = std::clamp<long>(bias, 20, max_pow - 20);
                d = (bias > max_pow / 2) ? max_pow - 1 - bias : bias;

                SERIAL_ECHOLNPAIR(" bias: ", bias, " d: ", d, " min: ", min_t, " max: ", max_t);

                if (cycles > 2 && max_t - min_t > 0.1f) {
                    const float ku = (4.0f * d) / (std::numbers::pi_v<float> * (max_t - min_t) * 0.5f);
                    const float tu = float(t_low + t_high) * 0.001f;
                    const float kp = 0.6f * ku;
                    const float ki = 2 * kp / tu;
                    const float kd = kp * tu * 0.125f;
                    SERIAL_ECHOLNPAIR(" Ku: ", ku, " Tu: ", tu, " Kp: ", kp, " Ki: ", ki, " Kd: ", kd);

                    tuned.Kp = kp;
                    tuned.Ki = scalePID_i(ki);
                    tuned.Kd = scalePID_d(kd);
                    has_result = true;
                }
            }

            set_heater(bias + d);
            cycles++;
            min_t = target;

            char text[40];
            snprintf(text, sizeof(text), "PID autotune: cycle %d/%d", std::min(cycles, ncycles), ncycles);
            status.update<PrintStatusMessage::custom>(make_message(text));
        }

        if (current > abort_temp) {
            return std::unexpected("temperature too high");
        }

        if (ELAPSED(now, (heating ? t2 : t1) + max_relay_phase_ms)) {
            return std::unexpected("target not crossed, timed out");
        }

        if (cycles > ncycles && cycles > 2) {
            break;
        }
    }

    if (!has_result) {
        return std::unexpected("no oscillation measured");
    }
    return tuned;
}

} // namespace
    #endif

/** \addtogroup G-Codes
 * @{
 */

/**
 *### M303: Run PID tuning <a href="https://reprap.org/wiki/G-code#M303:_Run_PID_tuning">M303: Run PID tuning</a>
 *
 * Relay autotune of the nozzle heater. The heater is turned off when done.
 * Only available with nozzle PID autotune support.
 *
 *#### Usage
 *
 *    M303 [ E | S | C | U ]
 *
 *#### Parameters
 *
 * - `E` - Tool to tune (default 0); the bed is not supported
 * - `S` - Target temperature (default 200), up to the hotend's highest settable temperature
 * - `C` - Number of cycles (default 5, 3 to 20)
 * - `U` - Apply and save the result (default 0)
 *
 *#### Examples
 *
 *    M303 S400 C8 U1 ; Tune at 400 °C for 8 cycles, apply and save the result
 */
void GcodeSuite::M303() {
    #if HAS_NOZZLE_PID_AUTOTUNE()
    GCodeParser2 p;
    if (!p.parse_marlin_command()) {
        return;
    }

    const auto e = p.option<int16_t>('E').value_or(0);
    if (e < 0 || e >= PhysicalToolIndex::count) {
        SERIAL_ERROR_MSG("M303: only the nozzle can be tuned");
        return;
    }
    const auto tool = PhysicalToolIndex::from_raw(e);

    const int16_t max_target_temp = Hotend::for_tool(tool).max_nozzle_temp() - HEATER_MAXTEMP_SAFETY_MARGIN;
    const int16_t target = p.option<int16_t>('S').value_or(default_target_temp);
    if (target < min_target_temp || target > max_target_temp) {
        SERIAL_ERROR_START();
        SERIAL_ECHOLNPAIR("M303: S must be between ", min_target_temp, " and ", max_target_temp);
        return;
    }

    const int ncycles = std::clamp<int>(p.option<int>('C').value_or(default_cycles), min_cycles, max_cycles);
    const bool apply = p.option<bool>('U').value_or(false);

    buddy::SafetyTimerBlocker safety_timer_blocker;
    ScopeGuard heater_off = [&] {
        Hotend::for_tool(tool).set_nozzle_heater_pwm_override(std::nullopt);
        thermalManager.setTargetHotend(0, tool);
    };

    SERIAL_ECHOLNPGM("PID Autotune start");

    std::expected<HotendPIDConfig, const char *> result;
    {
        PrintStatusMessageGuard status;
        status.update<PrintStatusMessage::custom>(make_message("PID autotune: heating"));
        result = autotune_nozzle(tool, target, ncycles, status);
    }

    char text[64];
    if (!result) {
        snprintf(text, sizeof(text), "PID autotune failed: %s", result.error());
        SERIAL_ERROR_START();
        SERIAL_ECHOLN(text);

    } else {
        const float kp = result->Kp;
        const float ki = unscalePID_i(result->Ki);
        const float kd = unscalePID_d(result->Kd);
        SERIAL_ECHOLNPGM("PID Autotune finished!");
        SERIAL_ECHOLNPAIR("M301 P", kp, " I", ki, " D", kd);

        if (apply) {
            Hotend::for_tool(tool).set_nozzle_pid_config(*result);
            nozzle_pid_store::save(*result);
        }
        snprintf(text, sizeof(text), "PID P%.2f I%.2f D%.2f %s", (double)kp, (double)ki, (double)kd, apply ? "saved" : "(U1 to save)");
    }
    print_status_message().show_temporary<PrintStatusMessage::custom>(make_message(text), result_message_duration_ms);
    #else
    // Not supported, the code was a mess, sorry guys
    SERIAL_ECHOLNPGM("M303 is not supported");
    #endif
}

/** @}*/

#endif // HAS_PID_HEATING
