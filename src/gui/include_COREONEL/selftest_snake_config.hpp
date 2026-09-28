#pragma once
#include <utility_extensions.hpp>
#include <printer_selftest.hpp>
#include <option/has_precise_homing_corexy.h>
#include <selftest_types.hpp>
#include <utils/storage/enum_bitset.hpp>
#include <bsod/bsod.h>

namespace SelftestSnake {

// Order matters, snake and will be run in the same order, as well as menu items (with indices) will be
enum class Action {
    DoorSensor,
    Fans,
    Heaters,
    XCheck,
    YCheck,
    ZAlign, // also known as z_calib
    BeltTuning,
    Gears,
    FilamentSensorCalibration,
#if HAS_PRECISE_HOMING_COREXY()
    PreciseHoming,
#endif
    Loadcell, // Check loadcell before Z test, because it is used there
    ZCheck,
    PhaseSteppingCalibration,
    _count,
    _last = _count - 1,
    _first = DoorSensor,
};

/// Calibrations are optional and can be run in any order; the wizard runs them in the order of Action.
constexpr EnumBitset<Action, Action::_count> get_dependencies(Action) {
    return {};
}

TestResult get_test_result(Action action, ToolMask tool);
uint64_t get_test_mask(Action action);
} // namespace SelftestSnake
