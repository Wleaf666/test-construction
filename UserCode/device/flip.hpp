#pragma once

#include "command.hpp"
#include "dji.hpp"
#include "motor_trajectory.hpp"
#include "motor_vel_controller.hpp"

namespace device
{
class Flip final
{
public:
    Flip(const motors::DJIMotor::Config&                      motor_config,
         const PIDMotor::Config&                              velocity_pid,
         const PD::Config&                                    position_pd,
         const trajectory::MotorTrajectory<1>::ProfileConfig& profile);
    Flip(const Flip&)            = delete;
    Flip& operator=(const Flip&) = delete;

    void commandUpdate();                 // 命令任务，约10ms调用一次。
    void profileUpdate(float dt = 0.01f); // 100Hz：轨迹及到位判断。
    void errorUpdate();                   // 500Hz：位置PD及位置观测。
    void controllerUpdate();              // 1kHz：速度PID；CAN由app统一发送。

    volatile Command command      = Command::None;
    volatile float   target_deg   = 0.0f; // 相对人工零点的绝对角度，只接受0或180度。
    volatile Result  result       = Result::Ok;
    volatile bool    zeroed       = false;
    volatile bool    busy         = false;
    volatile float   position_deg = 0.0f; // 只读反馈。

private:
    void disconnect();
    // 外接减速比通过motor_config配置，反馈和目标均为夹爪端角度。
    motors::DJIMotor                motor_;
    controllers::MotorVelController controller_;
    trajectory::MotorTrajectory<1>  trajectory_;
    volatile bool                   command_active_ = false;
};
} // namespace device
