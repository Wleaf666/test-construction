#include "flip.hpp"
#include "isr_lock.h"

#include <cmath>

namespace device
{
Flip::Flip(const motors::DJIMotor::Config&                      motor_config,
           const PIDMotor::Config&                              velocity_pid,
           const PD::Config&                                    position_pd,
           const trajectory::MotorTrajectory<1>::ProfileConfig& profile) :
    motor_(motor_config),
    controller_(&motor_, { velocity_pid, controllers::ControlMode::ExternalPID }),
    trajectory_(&controller_, profile, position_pd, 0.5f)
{
}

void Flip::disconnect()
{
    if (trajectory_.enabled())
        trajectory_.disable();
    motor_.setCurrent(0);
    zeroed = busy = false;
}

void Flip::commandUpdate()
{
    const Command requested = command;
    if (requested == Command::None)
        return;
    command_active_ = true;
    __DMB();
    if (!motor_.isConnected())
    {
        disconnect();
        result = Result::OfflineOrNotZeroed;
    }
    else
    {
        switch (requested)
        {
        case Command::SetZero:
            if (trajectory_.enabled() || std::fabs(motor_.getVelocity()) > 0.1f)
                result = Result::BusyOrNotStationary;
            else
            {
                ISRGuard guard; // 仅保护设零：CAN接收中断也会写角度和圈数。
                motor_.resetAngle();
                zeroed = true;
                result = Result::Ok;
            }
            break;
        case Command::Enable:
            if (!zeroed)
                result = Result::OfflineOrNotZeroed;
            else if (busy)
                result = Result::BusyOrNotStationary;
            else
            {
                controller_.getPID().reset();
                result = trajectory_.enable() ? Result::Ok : Result::NotEnabled;
            }
            break;
        case Command::Disable:
            trajectory_.disable();
            motor_.setCurrent(0);
            busy   = false;
            result = Result::Ok;
            break;
        case Command::Move:
        {
            const float value = target_deg;
            if (!trajectory_.enabled())
                result = Result::NotEnabled;
            else if (busy)
                result = Result::BusyOrNotStationary;
            else if (value != 0.0f && value != 180.0f)
                result = Result::InvalidCommandOrValue;
            else
            {
                trajectory_.stop();
                busy   = trajectory_.setTarget(value);
                result = busy ? Result::Ok : Result::PlanningFailed;
            }
            break;
        }
        case Command::Stop:
            if (!trajectory_.enabled())
                result = Result::NotEnabled;
            else
            {
                trajectory_.stop();
                busy   = false;
                result = Result::Ok;
            }
            break;
        default:
            result = Result::InvalidCommandOrValue;
            break;
        }
    }
    __DMB();
    command_active_ = false;
    command         = Command::None;
}

void Flip::profileUpdate(float dt)
{
    if (command_active_)
        return;
    if (!motor_.isConnected())
    {
        disconnect();
        return;
    }
    trajectory_.profileUpdate(dt);
    if (busy && trajectory_.isFinished() && std::fabs(motor_.getVelocity()) < 0.1f)
        busy = false;
}

void Flip::errorUpdate()
{
    if (command_active_)
        return;
    trajectory_.errorUpdate();
    position_deg = motor_.getAngle();
}

void Flip::controllerUpdate()
{
    if (!command_active_)
        trajectory_.controllerUpdate();
}
} // namespace device
