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
    void profileUpdate(float dt = 0.01f); // 轨迹时间步进及到位检查，dt必须与调用周期一致。
    void errorUpdate();                   // 位置PD及位置观测，调用频率由app定时器决定。
    void controllerUpdate();              // 速度PID，当前app以1kHz调用；CAN由app统一发送。

    volatile Command command      = Command::None;
    volatile float   target_deg   = 0.0f; // 相对人工零点的绝对角度，只接受0或180度。
    volatile Result  result       = Result::Ok;
    volatile bool    zeroed       = false; // 人工设零后有效；失联后失效。
    volatile bool    busy         = false; // 正常到位、Stop、Disable或失联均会清零。
    volatile float   position_deg = 0.0f; // 仅观察：经减速比换算的角度，不含夹爪端间隙测量。

private:
    void disconnect();
    // 外接减速比通过motor_config配置，反馈和目标均为夹爪端角度。
    motors::DJIMotor                motor_;
    controllers::MotorVelController controller_;
    trajectory::MotorTrajectory<1>  trajectory_;
    volatile bool                   command_active_ = false;
};
} // namespace device
