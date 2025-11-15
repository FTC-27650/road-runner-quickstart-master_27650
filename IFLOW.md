# IFLOW.md - Road Runner Quickstart 项目指南

## 项目概述

这是一个基于 Road Runner 的 FIRST Tech Challenge (FTC) 机器人驱动程序开发框架。Road Runner 是一个先进的
FTC 机器人运动规划和控制系统，提供精确的路径规划、轨迹跟随和本地化功能。

该项目包含以下主要组件：

- **MecanumDrive** 和 **TankDrive** - 用于全向和差动驱动机器人的驱动控制类
- **本地化系统** - 包括双轮、三轮、OTOS 和 Pinpoint 编码器本地化
- **调优工具** - 用于校准和优化机器人性能的 OpMode
- **MeepMeepTesting** - 用于在模拟环境中测试轨迹的模块

## 构建和运行

### 环境要求

- Android Studio
- FTC SDK 11.0.0
- Gradle

### 构建命令

```bash
# 构建项目
./gradlew build

# 安装到机器人控制器
./gradlew :TeamCode:installDebug
```

### 依赖项

项目使用以下关键依赖：

- `com.acmerobotics.roadrunner:ftc:0.1.25` - Road Runner FTC 集成
- `com.acmerobotics.roadrunner:core:1.0.1` - Road Runner 核心库
- `com.acmerobotics.roadrunner:actions:1.0.1` - Road Runner 动作系统
- `com.acmerobotics.dashboard:dashboard:0.5.0` - 遥测仪表板
- `com.bylazar:fullpanels:1.0.6` - 额外的面板功能
- `org.firstinspires.ftc:RobotCore:11.0.0` - FTC 核心库

## 项目结构

### FtcRobotController

包含 FTC 机器人的基本控制代码和示例 OpMode。

### TeamCode

主要的团队代码目录，包含：

- `MecanumDrive.java` - 全向驱动控制实现
- `TankDrive.java` - 差动驱动控制实现
- `Localizer.java` - 本地化系统接口
- `tuning/` - 包含各种调优工具的目录
- `messages/` - 用于仪表板通信的消息类
- `FTC_27650_TeleOp/` - 27650团队的特定TeleOp实现
- `ftc27650_test/` - 27650团队的测试文件
- `OTOSLocalizer.java` - OTOS (Optical Tracking Odometry System) 本地化器
- `PinpointLocalizer.java` - GoBilda Pinpoint IMU 本地化器
- `ThreeDeadWheelLocalizer.java` - 三轮死轮本地化器
- `TwoDeadWheelLocalizer.java` - 双轮死轮本地化器

### MeepMeepTesting

用于在计算机上测试轨迹的模拟环境。

## 本地化系统

项目支持多种本地化方法：

- **MecanumDrive.DriveLocalizer** - 使用驱动轮编码器进行本地化
- **TankDrive.DriveLocalizer** - 使用驱动轮编码器进行本地化
- **TwoDeadWheelLocalizer** - 使用两个正交编码器轮进行本地化
- **ThreeDeadWheelLocalizer** - 使用三个编码器轮进行本地化
- **OTOSLocalizer** - 使用 SparkFun OTOS 光学编码器进行本地化
- **PinpointLocalizer** - 使用 GoBilda Pinpoint IMU 进行本地化

## 开发约定

### 配置参数

在 `MecanumDrive.java` 和 `TankDrive.java` 中，通过 `Params` 类配置机器人的物理参数，如：

- 轮子直径和编码器刻度
- 轨道宽度
- 电机前馈参数 (kS, kV, kA)
- 速度和加速度限制
- 控制器增益

### 调优

使用 `TuningOpModes.java` 中的调优工具来校准机器人的运动参数，包括：

- 前馈调优 (Feedforward tuning)
- 反馈调优 (Feedback tuning)
- 本地化校准
- 方向调试工具

### 轨迹规划

使用 `TrajectoryActionBuilder` 创建复杂的轨迹，支持：

- 直线移动 (lineTo)
- 样条曲线 (splineTo)
- 旋转动作 (turn)
- 速度和加速度约束

## 使用说明

### 基本设置

1. 在硬件映射中配置电机名称 (Mecanum: leftFront, leftBack, rightFront, rightBack; Tank: left, right)
2. 在 `MecanumDrive.Params` 或 `TankDrive.Params` 中设置机器人的物理参数
3. 配置 IMU 方向参数
4. 设置编码器参数和本地化系统

### 轨迹执行

1. 使用 `actionBuilder` 创建轨迹
2. 使用 `Actions.runBlocking()` 执行轨迹
3. 通过 Dashboard 仪表板监控机器人的位置和性能

### 调优流程

1. 运行 `ManualFeedforwardTuner` 进行前馈参数调优
2. 运行 `ManualFeedbackTuner` 进行反馈控制器调优
3. 使用 `LocalizationTest` 验证本地化精度
4. 使用 `SplineTest` 测试轨迹执行效果

### 团队特定实现

此项目包含 FTC 27650 的特定实现，位于 `FTC_27650_TeleOp` 目录中，包括：

- `MecanumWheel_new_4.java` - 针对27650团队的TeleOp实现
- `MyRobotHardware_27650_TeleOp.java` - 27650团队的硬件映射

## 测试和验证

### 调优OpMode

- `ForwardRampLogger` - 前进加速度调优
- `LateralRampLogger` - 侧向加速度调优
- `AngularRampLogger` - 角加速度调优
- `ForwardPushTest` - 前进摩擦力测试
- `LateralPushTest` - 侧向摩擦力测试

### 本地化测试

- `LocalizationTest` - 本地化精度验证
- `SplineTest` - 轨迹执行测试

## 文档

详细文档可在 https://rr.brott.dev/docs/v1-0/tuning/ 找到。