#pragma once

#include <cstdint>

namespace device
{
// Ozone：先修改距离/角度，再写command；处理一次后自动回到None。
enum class Command : uint32_t
{
    None    = 0,
    SetZero = 1,
    Enable  = 2,
    Disable = 3,
    Move    = 4,
    Stop    = 5 // 取消轨迹并保持当前位置，不是S曲线减速停车。
};

enum class Result : uint32_t
{
    Ok                    = 0,
    OfflineOrNotZeroed    = 1,
    NotEnabled            = 2,
    InvalidCommandOrValue = 3,
    BusyOrNotStationary   = 4,
    PlanningFailed        = 5
};
} // namespace device
