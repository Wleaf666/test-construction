#pragma once

#include "command.hpp"
#include "dji.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

namespace device
{
class Lift final
{
public:
    Lift(const motors::DJIMotor::Config&                      motor_config,
         const PIDMotor::Config&                              velocity_pid,
         const PD::Config&                                    position_pd,
         const trajectory::MotorTrajectory<1>::ProfileConfig& profile);
    Lift(const Lift&)            = delete;
    Lift& operator=(const Lift&) = delete;

    void commandUpdate();                 // 命令任务，约10ms调用一次。
    void profileUpdate(float dt = 0.01f); // 100Hz：轨迹及到位判断。
    void errorUpdate();                   // 500Hz：位置PD及位置观测。
    void controllerUpdate();              // 1kHz：速度PID；CAN由app统一发送。

    volatile Command command     = Command::None;
    volatile float   distance_mm = 0.0f; // 相对位移，mm；方向由motor_config.reverse决定。
    volatile Result  result      = Result::Ok;
    volatile bool    zeroed      = false;
    volatile bool    busy        = false;
    volatile float   position_mm = 0.0f; // 只读反馈。

private:
    void                            disconnect();
    static constexpr float          degrees_per_mm = 360.0f / (5.0f * 32.0f);
    motors::DJIMotor                motor_;
    controllers::MotorVelController controller_;
    trajectory::MotorTrajectory<1>  trajectory_;
    volatile bool                   command_active_ = false;
};
} // namespace device
