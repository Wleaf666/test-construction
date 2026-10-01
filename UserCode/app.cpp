#include "can.h"
#include "can_driver.h"
#include "cmsis_os2.h"
#include "device/flip.hpp"
#include "device/lift.hpp"
#include "gpio_driver.hpp"
#include "main.h"
#include "tim.h"
#include "watchdog.hpp"

namespace
{
const bsp::gpio::GpioPinOutput fixed_gripper_gpio{ fixed_hripper_GPIO_Port, fixed_hripper_Pin };
const bsp::gpio::GpioPinOutput moving_gripper_gpio{ moving_hripper_GPIO_Port, moving_hripper_Pin };
} // namespace

// 应用只负责参数、设备实例及调度。CubeMX负责外设和中断优先级配置。
device::Lift* lift = nullptr;
device::Flip* flip = nullptr;

// 保留当前调参值：速度PID 1kHz，位置PD 500Hz，轨迹100Hz。
// 参数在实例构造时复制到库；初始化后修改这些变量不会自动更新设备内部参数。
PIDMotor::Config                              lift_pid{ 200.0f, 3.0f, 0.0f, 16384.0f * 0.75f };
PIDMotor::Config                              flip_pid{ 200.0f, 3.0f, 0.0f, 16384.0f * 0.75f };
PD::Config                                    lift_pd{ 5.0f, 25.0f, 450.0f };
PD::Config                                    flip_pd{ 5.0f, 25.0f, 450.0f };
trajectory::MotorTrajectory<1>::ProfileConfig lift_profile{ 22.5f, 45.0f, 225.0f };
trajectory::MotorTrajectory<1>::ProfileConfig flip_profile{ 30.0f, 60.0f, 300.0f };

// Ozone示例：先设零、使能，再运动；等command回到None再提交下一条。
// lift->command = device::Command::SetZero; // 1
// lift->command = device::Command::Enable;  // 2
// lift->distance_mm = 20.0f;
// lift->command = device::Command::Move;    // 4，相对上移20mm（先验证方向）
// flip->target_deg = 180.0f;                // 回程写0
// flip->command = device::Command::Move;
// 其余命令：None=0、Disable=3、Stop=5。Stop是当前位置保持，不是减速轨迹。
// 观察lift/flip的result、zeroed、busy，以及position_mm / position_deg。
volatile bool fixed_gripper_closed  = false;
volatile bool moving_gripper_closed = false;

void can_receive(const CAN_HandleTypeDef*   hcan,
                 const CAN_RxHeaderTypeDef* header,
                 const uint8_t*             data)
{
    if (header->IDE == CAN_ID_STD && header->RTR == CAN_RTR_DATA && header->DLC == 8)
        motors::DJIMotor::CANBaseReceiveCallback(hcan, header, data);
}

void can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    CAN_InitMainCallback(&hcan1);
    CAN_RegisterCallback(&hcan1, can_receive);
}

void motor_init()
{
    lift = new device::Lift({ &hcan1, motors::DJIMotor::Type::M3508_C620, 1, false, false, 1.0f },
                            lift_pid,
                            lift_pd,
                            lift_profile);
    flip = new device::Flip(
            { &hcan1, motors::DJIMotor::Type::M3508_C620, 2, false, false, 34.0f / 24.0f },
            flip_pid,
            flip_pd,
            flip_profile);
}

void gripper_update()
{
    fixed_gripper_gpio.write(fixed_gripper_closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
    moving_gripper_gpio.write(moving_gripper_closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motorProfileUpadta_Tim100(TIM_HandleTypeDef*)
{
    lift->profileUpdate(0.01f);
    flip->profileUpdate(0.01f);
}

void motorCurrentUpadta_Tim1k(TIM_HandleTypeDef*)
{
    service::Watchdog::EatAll();
    lift->controllerUpdate();
    flip->controllerUpdate();
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}

void motorErrorUpadta_Tim500(TIM_HandleTypeDef*)
{
    lift->errorUpdate();
    flip->errorUpdate();
}

void tim_init()
{
    HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorProfileUpadta_Tim100);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorCurrentUpadta_Tim1k);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorErrorUpadta_Tim500);
    HAL_TIM_Base_Start_IT(&htim3);
}

void liftCommandUpdate(void*)
{
    for (;;)
    {
        lift->commandUpdate();
        osDelay(10);
    }
}

void flipCommandUpdate(void*)
{
    for (;;)
    {
        flip->commandUpdate();
        osDelay(10);
    }
}

void gripperCommandUpdate(void*)
{
    for (;;)
    {
        gripper_update();
        osDelay(10);
    }
}

extern "C" void Init(void*)
{
    can_init();
    motor_init();
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    tim_init();

    // 两个命令任务需要执行S曲线规划，明确预留栈空间；任务只在Init中创建。
    osThreadAttr_t command_task_attr{};
    command_task_attr.priority   = osPriorityNormal;
    command_task_attr.stack_size = 4096;
    command_task_attr.name       = "lift_command";
    const auto lift_task         = osThreadNew(liftCommandUpdate, nullptr, &command_task_attr);
    command_task_attr.name       = "flip_command";
    const auto flip_task         = osThreadNew(flipCommandUpdate, nullptr, &command_task_attr);
    command_task_attr.name       = "gripper_command";
    command_task_attr.stack_size = 512;
    const auto gripper_task      = osThreadNew(gripperCommandUpdate, nullptr, &command_task_attr);
    if (lift_task == nullptr || flip_task == nullptr || gripper_task == nullptr)
    {
        HAL_TIM_Base_Stop_IT(&htim2);
        HAL_TIM_Base_Stop_IT(&htim3);
        HAL_TIM_Base_Stop_IT(&htim6);
        HAL_CAN_Stop(&hcan1);
        Error_Handler();
        return;
    }
    osThreadExit();
}
