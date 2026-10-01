#include "can.h"
#include "can_driver.h"
#include "cmsis_os2.h"
#include "device/flip.hpp"
#include "device/lift.hpp"
#include "gpio_driver.hpp"
#include "main.h"
#include "tim.h"
#include "watchdog.hpp"

// 应用侧GPIO句柄使用的引脚宏，仅在本编译单元生效。
// Core/Src/gpio.c仍需CubeMX在main.h生成同名引脚宏，不能由此处替代。
#define fixed_hripper_Pin        GPIO_PIN_0
#define fixed_hripper_GPIO_Port  GPIOC
#define moving_hripper_Pin       GPIO_PIN_1
#define moving_hripper_GPIO_Port GPIOC

// 句柄只保存端口和引脚；时钟、模式及初始低电平仍由MX_GPIO_Init配置。
bsp::gpio::GpioPinOutput fixed_gripper_gpio{ fixed_hripper_GPIO_Port, fixed_hripper_Pin };
bsp::gpio::GpioPinOutput moving_gripper_gpio{ moving_hripper_GPIO_Port, moving_hripper_Pin };

// 应用只负责参数、设备实例及调度。CubeMX负责外设和中断优先级配置。
device::Lift* lift = nullptr;
device::Flip* flip = nullptr;

// 当前生成配置：TIM6速度PID/CAN发送1kHz，TIM3位置PD实际200Hz，TIM2轨迹100Hz。
// 若要位置PD 500Hz，在CubeMX将TIM3的ARR改为2000-1；回调名称不决定频率。
// 以下为当前调参值，并非实物验证结论。构造时复制到库，运行中改配置变量不会自动生效。
PIDMotor::Config lift_pid{ 200.0f, 3.0f, 0.0f, 16384.0f * 0.75f };
PIDMotor::Config flip_pid{ 200.0f, 3.0f, 0.0f, 16384.0f * 0.75f };
// 速度环电流上限为12288个C620协议单位，对应15A转矩电流指令，不是电源输入电流。
PD::Config lift_pd{ 5.0f, 25.0f, 450.0f };
PD::Config flip_pd{ 5.0f, 25.0f, 450.0f };
// 轨迹参数单位依次为deg/s、deg/s²、deg/s³；升降按2.25deg/mm换算。
trajectory::MotorTrajectory<1>::ProfileConfig lift_profile{ 22.5f, 45.0f, 225.0f };
trajectory::MotorTrajectory<1>::ProfileConfig flip_profile{ 30.0f, 60.0f, 300.0f };

// Ozone：每台设备分别在静止、失能状态设零，再使能；每条命令处理清零后再发下一条。
// lift->command = device::Command::SetZero; // 1
// lift->command = device::Command::Enable;  // 2
// lift->distance_mm = 20.0f;
// lift->command = device::Command::Move;    // 4，相对上移20mm（先验证方向）
// flip->command = device::Command::SetZero;
// flip->command = device::Command::Enable;
// flip->target_deg = 180.0f;                // 回程写0
// flip->command = device::Command::Move;
// 其余命令：None=0、Disable=3、Stop=5。Stop是当前位置保持，不是减速轨迹。
// result=Ok仅表示命令接受；正常到位后busy清零，但Stop/Disable/失联也会清零。
// 联合观察result、zeroed、busy及position_mm / position_deg，不能只凭busy判断到位。
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
// true输出高电平使阀通电闭合，false输出低电平打开；变量不是气缸到位反馈。

void gripper_update()
{
    fixed_gripper_gpio.write(fixed_gripper_closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
    moving_gripper_gpio.write(moving_gripper_closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// TIM2当前100Hz，轨迹每次推进10ms，并检查位置误差和速度是否满足到位条件。
void motorProfileUpadta_Tim100HZ(TIM_HandleTypeDef*)
{
    lift->profileUpdate(0.01f);
    flip->profileUpdate(0.01f);
}

// TIM6当前1kHz，软件看门狗每1ms递减，两台速度PID更新后聚合发送一帧电流。
void motorCurrentUpadta_Tim1kHZ(TIM_HandleTypeDef*)
{
    service::Watchdog::EatAll();
    lift->controllerUpdate();
    flip->controllerUpdate();
    motors::DJIMotor::SendIqCommand(&hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}

// 保留现有函数名；实际频率取决于TIM3，当前生成配置为200Hz而非500Hz。
void motorErrorUpadta_Tim500HZ(TIM_HandleTypeDef*)
{
    lift->errorUpdate();
    flip->errorUpdate();
}

void tim_init()
{
    HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorProfileUpadta_Tim100HZ);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorCurrentUpadta_Tim1kHZ);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_RegisterCallback(&htim3, HAL_TIM_PERIOD_ELAPSED_CB_ID, motorErrorUpadta_Tim500HZ);
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

    // S曲线规划在命令任务执行，不在定时器中断规划；栈大小以字节计，仍需上板检查余量。
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
