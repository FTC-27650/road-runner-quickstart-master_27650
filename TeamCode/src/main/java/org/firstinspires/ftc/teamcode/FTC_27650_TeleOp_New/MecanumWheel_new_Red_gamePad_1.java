package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp_New;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * FTC 27650 红队手动控制模式
 * 实现了Mecanum轮全向移动、球类处理、视觉识别、自动瞄准等功能
 */
@TeleOp(name = "手动27650_new_red_手柄1", group = "LinearOpmode")
@Config
public class MecanumWheel_new_Red_gamePad_1 extends LinearOpMode {

    // 旋转电机PID控制参数
    public static int ROTATE_MOTOR_MAX_ERROR_POSITION = 100;
    public static int ROTATION_ERROR_POSITION = 1422;
    public static double ROTATE_KP = 0.00035, ROTATE_KI = 0.00005, ROTATE_KD = 0.000008, ROTATE_KI_MAX = 5000, ROTATE_KF = 0;
    public static double ROTATE_MOTOR_MAX_POWER = 0.8;
    public static double ROTATE_MOTOR_MIN_POWER = 0.05;

    // 舵机相关参数
    public static volatile double STRIKER_SERVO_DOWN_POSITION = 0.54;
    public static volatile double STRIKER_SERVO_UP_POSITION = 0.37;
    public static volatile double FLYWHEEL_TARGET_VELOCITY = 0;
    public volatile static double XI_MOTOR_MIN_POWER = 0;
    public static int COLOR_DETECTION_DELAY_TIME = 200;
    public static int ROTATION_TIME_DELAY = 450;
    public static int STRIKER_UP_TIME = 250;
    public static int STRIKER_DOWN_TIME = 250;
    public static double TURRET_TARGET_ANGLE_FAR = -7.5; // 远距离目标角度
    public static double TURRET_TARGET_ANGLE_NEAR = -3;   // 近距离目标角度
    public static double TURRET_MIN_POWER = 0;
    // HSV颜色值数组
    final float[] hsvValuesFront = new float[3]; // 前方颜色传感器HSV值
    final float[] hsvValuesLeft = new float[3];  // 左方颜色传感器HSV值
    final float[] hsvValuesRight = new float[3]; // 右方颜色传感器HSV值
    private final AtomicReference<Double> angleServoPositionRef = new AtomicReference<>(0.0); // 射球角度，使用AtomicReference保证线程安全

    private final ElapsedTime runtime = new ElapsedTime();
    // 线程实例
    private final MecanumThread mecanumThread = new MecanumThread();
    private final OtherThread otherThread = new OtherThread();
    private final MyRobotHardware_27650_TeleOp_new robot = new MyRobotHardware_27650_TeleOp_new(this);
    private final LedControlThread ledControlThread = new LedControlThread();
    private final SetFlyWheelVelocityThread setFlyWheelVelocityThread = new SetFlyWheelVelocityThread();
    private final SetRotateMotorPositionThread setRotateMotorPositionThread = new SetRotateMotorPositionThread();
    private final LimeLightThread limeLightThread = new LimeLightThread();
    private final TurretTargetingRed turretTargeting = new TurretTargetingRed();
    private final Pose2dSetTurretPositionThread pose2dSetTurretPositionThread = new Pose2dSetTurretPositionThread();

    // 旋转电机控制变量
    volatile boolean isRotateMotorPositionThreadActive = true;
    long rotateMotorCurrentPosition = 0;
    long rotateMotorPreviousTargetPosition = 0;
    long rotateMotorTargetPosition = 0;
    double rotateMotorPower = 0;
    int rotationStep = 2731; // 旋转步长
    double GREEN_HUE_MIN = 140, GREEN_HUE_MAX = 180;
    double PURPLE_HUE_MIN = 215, PURPLE_HUE_MAX = 260;
    float colorSensorGain = 3; // 颜色传感器增益值，要>=1

    // 颜色检测结果
    volatile String colorFront = "无", colorLeft = "无", colorRight = "无";
    volatile double distanceFront = 0, distanceLeft = 0, distanceRight = 0;
    volatile double strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;

    // 角度伺服速度
    double angleServoSpeed = 0.02;

    // 飞轮PID参数
    double FLYWHEEL_KP = 7, FLYWHEEL_KI = 1.5, FLYWHEEL_KD = 0.05;
    double FLYWHEEL_MAX_INTEGRAL = 600, FLYWHEEL_KF = 1;
    volatile double flywheelCurrentVelocity = 0;
    volatile double flywheelMaxVelocity = 2500;
    volatile double xiMotorPower = 0;

    // 炮台控制变量
    volatile double turretCurrentPosition = 0; // 炮台当前位置
    volatile int ballProcessingState = 1, autoRotationState = 1, ballCount = 0, greenBallState = 1, purpleBallState = 1;
    volatile long lastPressTime = 0;
    volatile boolean isLedColorActive = false;
    volatile boolean isTurretAutoAimingActive = false;

    // 机器人位姿
    volatile double pose2dX = 0, pose2dY = 0;
    volatile double pose2dXCM = 0, pose2dYCM = 0;
    volatile double distanceCarToRed = 0;

    // LimeLight反馈
    volatile double xDegrees = 0, yDegrees = 0, distance = 0;
    volatile int fiducialId = 0;

    // 炮台PID参数
    volatile double carHeading = 0; // 车辆IMU实时角度
    volatile double turretTargetAngle = 0; // 炮台目标角度
    double turretKp = 0, turretKi = 0, turretKd = 0, turretKiMax = 0, turretMinErrorSetZero = 0;
    double TURRET_KP_NOT_SEE = 0.02, TURRET_KI_NOT_SEE = 0.025, TURRET_KD_NOT_SEE = 0.00005,
            TURRET_KI_MAX_NOT_SEE = 20, TURRET_MIN_ERROR_SET_ZERO_NOT_SEE = 3;
    double TURRET_KP_SEE = 0.003, TURRET_KI_SEE = 0.003, TURRET_KD_SEE = 0.00005,
            TURRET_KI_MAX_SEE = 20, TURRET_MIN_ERROR_SET_ZERO_SEE = 3;
    volatile double turretPower = 0;

    // 初始位姿
    Pose2d initialPose = new Pose2d(60, -167.5, Math.toRadians(90));

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.setMsTransmissionInterval(11);
        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();
        mecanumInit();

        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();

        mecanumThread.start();
        otherThread.start();
        ledControlThread.start();
        setFlyWheelVelocityThread.start();
        setRotateMotorPositionThread.start();
        limeLightThread.start();
        pose2dSetTurretPositionThread.start();

        while (opModeIsActive()) {
            buttonControlRotateMotor();
        }
    }

    /**
     * Mecanum初始化
     * 执行旋转电机归零操作
     */
    public void mecanumInit() {
        isRotateMotorPositionThreadActive = false;
        robot.rotateMotor.setPower(0);
        strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
        robot.strikerServo.setPosition(strikerServoPosition);
        sleep(200);

        runtime.reset();
        while (runtime.seconds() <= 0.2) {
            robot.rotateMotor.setPower(-0.3);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);
        strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
        sleep(200);
        robot.strikerServo.setPosition(STRIKER_SERVO_DOWN_POSITION);
        robot.angleServo.setPosition(getAngleServoPosition());
        robot.leftTurretServo.setPosition(0.5);
        robot.rightTurretServo.setPosition(0.5);
        sleep(200);

        runtime.reset();
        while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 3) {
            robot.rotateMotor.setPower(0.5);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);

        while (robot.magnetic_in.isPressed()) {
            robot.rotateMotor.setPower(0.1);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);

        rotateMotorTargetPosition = 0;
        rotateMotorPreviousTargetPosition = 0;
        colorSensorGain = 2;
        robot.rotateMotorEncoderRest();
        ballProcessingState = 1;
        ballCount = 0;
        autoRotationState = 1;
        greenBallState = 1;
        purpleBallState = 1;
        isLedColorActive = false;
        sleep(300);
        isRotateMotorPositionThreadActive = true;
        setRotateMotorPositionThread.start();
    }

    /**
     * 显示系统状态信息
     */
    public void show() {
        telemetry.clear();
        telemetry.addData("imu", "%4.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("射球角度", "%4.2f", getAngleServoPosition());

        telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), flywheelCurrentVelocity);
        telemetry.addData("右飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelRight.getPower(), robot.flyWheelRight.getVelocity());
        telemetry.addData("旋吸功率", "%4.2f", xiMotorPower);

        telemetry.addData("id", "%d", fiducialId);
        telemetry.addData("x 角度偏移", "%f", xDegrees);
        telemetry.addData("y 角度偏移", "%f", yDegrees);
        telemetry.addData("distance", "%fCM", distance);

        telemetry.addData("车辆X坐标 厘米", pose2dXCM);
        telemetry.addData("车辆Y坐标 厘米", pose2dYCM);
        telemetry.addData("车辆角度（度）", carHeading); // 转成度更直观
        telemetry.addData("炮台实际位置", "%f", turretCurrentPosition);
        telemetry.addData("炮台目标角度(度)", turretTargetAngle);
        telemetry.addData("炮台功率", turretPower);
        telemetry.addData("a,b,c,g,p", "%d ,%d, %d, %d, %d", ballProcessingState, autoRotationState, ballCount, greenBallState, purpleBallState);
        telemetry.update();
    }

    /**
     * Mecanum轮驱动控制
     * 实现全向移动算法
     */
    public void mecanumDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        // 获取机器人当前航向角（弧度）
        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // 将输入坐标从机器人坐标系转换到世界坐标系
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        // 计算功率分母，确保功率不超过1
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        // 按下左保险杠时减速
        if (gamepad1.left_bumper) {
            flPower = flPower * 0.65;
            frPower = frPower * 0.65;
            brPower = brPower * 0.65;
            blPower = blPower * 0.65;
        }

        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.br.setPower(brPower);
        robot.bl.setPower(blPower);
    }

    /**
     * 伺服控制
     * 处理射球角度和打击机构
     */
    public void servoControl() {
        if (rotateMotorTargetPosition % rotationStep == 0) {
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;  // 一键下降
        }

        // 通过D-pad控制射球角度
        if (gamepad1.dpad_up) {
            updateAngleServoPosition(value -> Math.min(value + angleServoSpeed, 1));
        }
        if (gamepad1.dpad_down) {
            updateAngleServoPosition(value -> Math.max(value - angleServoSpeed, 0));
        }

        // 自动瞄准模式下的角度控制
        if (isTurretAutoAimingActive) {
            if (fiducialId == 24 && distance <= 255) {
                // 根据距离调整射球角度
                updateAngleServoPosition(value -> 0.004 * distance - 0.031);
            } else if (fiducialId == 0 && distanceCarToRed <= 48) {
                updateAngleServoPosition(value -> (double) 0);
            } else {
                updateAngleServoPosition(value -> 0.7);
            }
        }
        robot.strikerServo.setPosition(strikerServoPosition);
        robot.angleServo.setPosition(getAngleServoPosition());
    }

    /**
     * 颜色传感器处理
     * 识别球的颜色并测量距离
     */
    public void colorSensor() {
        if (gamepad1.a) colorSensorGain += 0.005F;
        else if (gamepad1.b && colorSensorGain > 1) colorSensorGain -= 0.005F;
        robot.colorSensorFront.setGain(colorSensorGain);
        robot.colorSensorLeft.setGain(colorSensorGain);
        robot.colorSensorRight.setGain(colorSensorGain);

        NormalizedRGBA colorsFront = robot.colorSensorFront.getNormalizedColors();
        NormalizedRGBA colorsLeft = robot.colorSensorLeft.getNormalizedColors();
        NormalizedRGBA colorsRight = robot.colorSensorRight.getNormalizedColors();

        Color.colorToHSV(colorsFront.toColor(), hsvValuesFront);
        Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
        Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);

        distanceFront = ((DistanceSensor) robot.colorSensorFront).getDistance(DistanceUnit.CM);
        distanceLeft = ((DistanceSensor) robot.colorSensorLeft).getDistance(DistanceUnit.CM);
        distanceRight = ((DistanceSensor) robot.colorSensorRight).getDistance(DistanceUnit.CM);

        // 前方颜色传感器检测
        if (distanceFront <= 3.5) {
            colorFront = "有";
        } else {
            colorFront = "无";
        }

        // 左方颜色传感器检测
        if (hsvValuesLeft[2] <= 0.01 && hsvValuesLeft[0] <= 130
                && (hsvValuesLeft[1] == 1 || hsvValuesLeft[1] == 0)) {
            colorLeft = "无";
        } else {
            if ((GREEN_HUE_MIN <= hsvValuesLeft[0] && hsvValuesLeft[0] <= GREEN_HUE_MAX)
                    && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95)
                    && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) {
                colorLeft = "green";
            } else if ((PURPLE_HUE_MIN <= hsvValuesLeft[0] && hsvValuesLeft[0] <= PURPLE_HUE_MAX)
                    && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95)
                    && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) {
                colorLeft = "purple";
            } else {
                colorLeft = "有";
            }
        }

        // 右方颜色传感器检测
        if (hsvValuesRight[2] <= 0.01 && hsvValuesRight[0] <= 130
                && (hsvValuesRight[1] == 1 || hsvValuesRight[1] == 0)) {
            colorRight = "无";
        } else {
            if ((GREEN_HUE_MIN <= hsvValuesRight[0] && hsvValuesRight[0] <= GREEN_HUE_MAX)
                    && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95)
                    && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) {
                colorRight = "green";
            } else if ((PURPLE_HUE_MIN <= hsvValuesRight[0] && hsvValuesRight[0] <= PURPLE_HUE_MAX)
                    && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95)
                    && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) {
                colorRight = "purple";
            } else {
                colorRight = "有";
            }
        }
    }

    /**
     * 吸球电机控制
     */
    public void xiMotor() {
        xiMotorPower = -gamepad2.left_stick_y + XI_MOTOR_MIN_POWER;
        robot.xiMotor.setPower(xiMotorPower);
    }

    /**
     * 发送LED脉冲信号
     */
    private void sendShortPulse(int widthMs) {
        robot.ledPin.setState(false);
        sleep(widthMs);
        robot.ledPin.setState(true);
    }

    /**
     * 按键控制旋转电机
     */
    public void buttonControlRotateMotor() {
        final int downTime = STRIKER_DOWN_TIME;

        // 控制进球转到下一步
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) sleep(10);
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(downTime);
            if (ballProcessingState == 1) rotateMotorTargetPosition += rotationStep;
            if (ballProcessingState == 2) rotateMotorTargetPosition += ROTATION_ERROR_POSITION;
            ballProcessingState = 1;
            autoRotationState = 1;
            greenBallState = 1;
            purpleBallState = 1;
            isLedColorActive = false;
        }

        // 控制进球装置进入自动旋转模式
        if (gamepad1.left_trigger > 0.1) {
            while (gamepad1.left_trigger > 0.1) sleep(10);
            isTurretAutoAimingActive = false;
            XI_MOTOR_MIN_POWER = 0.9;
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(downTime);
            if (ballProcessingState == 2) rotateMotorTargetPosition += ROTATION_ERROR_POSITION;
            ballProcessingState = 1;
            autoRotationState = 2;
            greenBallState = 1;
            purpleBallState = 1;
            isLedColorActive = false;
        }

        // 控制射球转到下一步
        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) sleep(10);
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(downTime);
            if (ballProcessingState == 2) rotateMotorTargetPosition += rotationStep;
            if (ballProcessingState == 1)
                rotateMotorTargetPosition += (rotationStep - ROTATION_ERROR_POSITION);
            ballProcessingState = 2;
            autoRotationState = 1;
            greenBallState = 1;
            purpleBallState = 1;
            isLedColorActive = true;
        }

        // 控制射球直到看到绿色球
        if (gamepad1.dpad_left) {
            while (gamepad1.dpad_left) sleep(10);
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(downTime);
            if (ballProcessingState == 2 && (colorLeft.equals("green") || colorRight.equals("green"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (ballProcessingState == 2) rotateMotorTargetPosition += rotationStep;
                if (ballProcessingState == 1)
                    rotateMotorTargetPosition += (rotationStep - ROTATION_ERROR_POSITION);
            }
            ballProcessingState = 2;
            autoRotationState = 1;
            greenBallState = 2;
            purpleBallState = 1;
            isLedColorActive = true;
        }

        // 控制射球直到看到紫色球
        if (gamepad1.dpad_right) {
            while (gamepad1.dpad_right) sleep(10);
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(downTime);
            if (ballProcessingState == 2 && (colorLeft.equals("purple") || colorRight.equals("purple"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (ballProcessingState == 2) rotateMotorTargetPosition += rotationStep;
                if (ballProcessingState == 1)
                    rotateMotorTargetPosition += (rotationStep - ROTATION_ERROR_POSITION);
            }
            ballProcessingState = 2;
            autoRotationState = 1;
            greenBallState = 1;
            purpleBallState = 2;
            isLedColorActive = true;
        }

        // 自动射三球
        if (gamepad1.y) {
            while (gamepad1.y) sleep(10);
            autoShoot();
        }

        // 手动抬自动落
        if (gamepad1.dpad_up) {
            while (gamepad1.dpad_up) {
                sleep(10);
            }
            strikerServoPosition = STRIKER_SERVO_UP_POSITION;
            sleep(STRIKER_UP_TIME);
            strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
            sleep(STRIKER_UP_TIME);
        }

        // 手动下降
        if (gamepad1.dpad_down) strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;

        // 启动自瞄系统
        if (gamepad1.a || ballCount == 3) {
            isTurretAutoAimingActive = true;
            XI_MOTOR_MIN_POWER = 0;
        }

        // 一键转盘初始化
        if (gamepad1.x) {
            while (gamepad1.x) sleep(10);
            mecanumInit();
        }
    }

    /**
     * 自动射球功能
     */
    public void autoShoot() {
        if (ballProcessingState == 1) {
            rotateMotorTargetPosition += (rotationStep - ROTATION_ERROR_POSITION);
            sleep(200);
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = STRIKER_SERVO_UP_POSITION;
                sleep(STRIKER_UP_TIME);
                strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
                sleep(STRIKER_DOWN_TIME);
                rotateMotorTargetPosition += rotationStep;
                sleep(ROTATION_TIME_DELAY);
            }
        }
        if (ballProcessingState == 2) {
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = STRIKER_SERVO_UP_POSITION;
                sleep(STRIKER_UP_TIME);
                strikerServoPosition = STRIKER_SERVO_DOWN_POSITION;
                sleep(STRIKER_DOWN_TIME);
                rotateMotorTargetPosition += rotationStep;
                sleep(ROTATION_TIME_DELAY);
            }
        }
        rotateMotorTargetPosition += ROTATION_ERROR_POSITION;
        ballProcessingState = 1;
        autoRotationState = 1;
        greenBallState = 1;
        purpleBallState = 1;
        ballCount = 0;
    }

    /**
     * 获取角度伺服位置
     */
    private double getAngleServoPosition() {
        return angleServoPositionRef.get();
    }

    /**
     * 安全更新角度伺服位置
     */
    private void updateAngleServoPosition(Function<Double, Double> updateFunction) {
        angleServoPositionRef.updateAndGet((UnaryOperator<Double>) updateFunction);
    }

    // 线程类定义
    public class MecanumThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                mecanumDrive();
            }
        }
    }

    public class OtherThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                servoControl();
                colorSensor();
                xiMotor();
                show();
            }
        }
    }

    public class LedControlThread extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
                    if ((colorLeft.equals("green") || colorRight.equals("green")) && isLedColorActive) {
                        sendShortPulse(60); // 45ms中脉冲 = 绿
                        lastPressTime = System.currentTimeMillis();
                    } else if ((colorLeft.equals("purple") || colorRight.equals("purple")) && isLedColorActive) {
                        sendShortPulse(40); // 20ms短脉冲 = 红
                        lastPressTime = System.currentTimeMillis();
                    } else if (ballCount == 3) {
                        sendShortPulse(20); // 70ms长脉冲 = 白
                        lastPressTime = System.currentTimeMillis();
                    } else {
                        sendShortPulse(80); // 120ms超长脉冲 = 关
                        lastPressTime = System.currentTimeMillis();
                    }
                }
                // 程序结束发送关灯脉冲
                sendShortPulse(120);
                robot.ledPin.setState(true);
                sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(e);
            }
        }
    }

    /**
     * 飞轮速度控制线程
     * 使用PID算法控制飞轮转速
     */
    public class SetFlyWheelVelocityThread extends Thread {
        public void run() {
            try {
                double error;
                double previousError = 0;
                double derivative;
                double integral = 0;
                double flywheelVelocity;

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期

                while (opModeIsActive()) {
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    // 根据自动瞄准状态设置飞轮目标速度
                    if (isTurretAutoAimingActive) {
                        if (distance <= 255) {
                            // 根据距离计算飞轮目标速度，确保速度≥1200
                            FLYWHEEL_TARGET_VELOCITY = Math.max(1200, 2.35 * distance + 1100);
                        } else {
                            FLYWHEEL_TARGET_VELOCITY = 1800;
                        }
                    } else {
                        FLYWHEEL_TARGET_VELOCITY = 900;
                    }

                    flywheelCurrentVelocity = robot.flyWheelLeft.getVelocity();

                    // PID控制算法
                    error = FLYWHEEL_TARGET_VELOCITY - flywheelCurrentVelocity;
                    integral += error * dt;

                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= 25) integral = 0;
                    integral = Math.max(-FLYWHEEL_MAX_INTEGRAL, Math.min(integral, FLYWHEEL_MAX_INTEGRAL));
                    derivative = (error - previousError) / dt;

                    flywheelVelocity = error * FLYWHEEL_KP + integral * FLYWHEEL_KI + derivative * FLYWHEEL_KD
                            + FLYWHEEL_TARGET_VELOCITY * FLYWHEEL_KF;

                    robot.flyWheelLeft.setVelocity(flywheelVelocity + (-gamepad2.right_stick_y * flywheelMaxVelocity));
                    robot.flyWheelRight.setVelocity(flywheelVelocity + (-gamepad2.right_stick_y * flywheelMaxVelocity));
                    previousError = error;

                    // 确保周期稳定
                    long sleepTime = Math.max(0, (long) ((loopPeriod - (loopTimer.seconds())) * 1000));
                    if (sleepTime > 0) {
                        sleep(sleepTime);
                    }
                }
                sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(e);
            }
        }
    }

    /**
     * 旋转电机位置控制线程
     * 使用PID算法控制旋转电机位置
     */
    public class SetRotateMotorPositionThread extends Thread {
        public void run() {
            try {
                double error;
                double previousError = 0;
                double derivative;
                double integral = 0;

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期

                while (opModeIsActive() && isRotateMotorPositionThreadActive) {
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();

                    // PID控制算法
                    error = rotateMotorTargetPosition - rotateMotorCurrentPosition;
                    integral += error * dt;

                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= ROTATE_MOTOR_MAX_ERROR_POSITION) integral = 0;
                    integral = Math.max(-ROTATE_KI_MAX, Math.min(integral, ROTATE_KI_MAX));
                    derivative = (error - previousError) / dt;
                    rotateMotorPower = error * ROTATE_KP + integral * ROTATE_KI + derivative * ROTATE_KD + rotateMotorTargetPosition * ROTATE_KF;
                    rotateMotorPower = rotateMotorPower * (12 / robot.batterySensor.getVoltage());

                    // 限制功率范围
                    rotateMotorPower = clampPower(rotateMotorPower);

                    // 根据不同状态处理位置更新
                    if (autoRotationState == 2 && Math.abs(error) <= ROTATE_MOTOR_MAX_ERROR_POSITION) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(100);
                        if (colorFront.equals("有") && ballCount < 3) {
                            ballCount++;
                            rotateMotorTargetPosition += rotationStep;
                            if (ballCount == 3) {
                                autoRotationState = 1;
                                rotateMotorTargetPosition += (rotationStep - ROTATION_ERROR_POSITION);
                                ballProcessingState = 2;
                            }
                        }
                    } else if (greenBallState == 2 && Math.abs(error) <= ROTATE_MOTOR_MAX_ERROR_POSITION) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(COLOR_DETECTION_DELAY_TIME);
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            greenBallState = 1;
                        } else {
                            rotateMotorTargetPosition += rotationStep;
                        }
                    } else if (purpleBallState == 2 && Math.abs(error) <= ROTATE_MOTOR_MAX_ERROR_POSITION) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(COLOR_DETECTION_DELAY_TIME);
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            purpleBallState = 1;
                        } else {
                            rotateMotorTargetPosition += rotationStep;
                        }
                    }
                    if (Math.abs(error) <= ROTATE_MOTOR_MAX_ERROR_POSITION) {
                        rotateMotorPower = 0;
                    }
                    robot.rotateMotor.setPower(rotateMotorPower);
                    previousError = error;

                    // 确保周期稳定
                    long sleepTime = Math.max(0, (long) ((loopPeriod - (loopTimer.seconds())) * 1000));
                    if (sleepTime > 0) {
                        sleep(sleepTime);
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(e);
            }
        }

        /**
         * 限制功率范围
         */
        private double clampPower(double power) {
            if (power >= ROTATE_MOTOR_MAX_POWER) return ROTATE_MOTOR_MAX_POWER;
            if (power > 0 && power <= ROTATE_MOTOR_MIN_POWER) return ROTATE_MOTOR_MIN_POWER;
            if (power < 0 && power >= -ROTATE_MOTOR_MIN_POWER) return -ROTATE_MOTOR_MIN_POWER;
            return Math.max(power, -ROTATE_MOTOR_MAX_POWER);
        }
    }

    /**
     * LimeLight线程
     * 处理视觉识别数据
     */
    public class LimeLightThread extends Thread {
        public void run() {
            // LimeLight安装角度（相对于垂直方向的偏转角度）
            double cameraAngle = 28;
            // LimeLight镜头中心到地面的距离（cm）
            double cameraHeight = 29.5;
            // 目标到地面的距离（cm）
            double targetHeight = 74.1;

            while (opModeIsActive()) {
                LLResult result = robot.limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        fiducialId = fr.getFiducialId();
                        xDegrees = fr.getTargetXDegrees();
                        yDegrees = fr.getTargetYDegrees();
                        // 计算机器人到目标的距离
                        distance = (targetHeight - cameraHeight) / Math.tan((cameraAngle + yDegrees) * (Math.PI / 180.0));
                    }
                } else {
                    fiducialId = 0;
                    xDegrees = 0;
                    yDegrees = 0;
                    distance = 0;
                }
            }
            robot.limelight.stop();
        }
    }

    /**
     * 位姿估计和炮台位置设置线程
     * 结合位姿估计和PID控制实现炮台自动瞄准
     */
    public class Pose2dSetTurretPositionThread extends Thread {
        double MAX_TURRET_DEAD_WHEEL_POSITION = 7042;
        double MAX_TURRET_ANGLE = 60;

        public void run() {
            try {
                MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

                double error;
                double previousError = 0;
                double derivative;
                double integral = 0;

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期

                while (opModeIsActive()) {
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    // 更新机器人位姿估计
                    drive.updatePoseEstimate();
                    Pose2d currentPose = drive.localizer.getPose();

                    // 获取机器人位置和角度
                    pose2dX = currentPose.position.x;
                    pose2dY = currentPose.position.y;
                    pose2dXCM = currentPose.position.x * 2.54;
                    pose2dYCM = currentPose.position.y * 2.54;
                    carHeading = Math.toDegrees(currentPose.heading.toDouble());
                    distanceCarToRed = Math.sqrt((-150.0 - pose2dX) * (-150.0 - pose2dX) + (pose2dYCM - 141.0) * (pose2dYCM - 141.0));

                    // 更新炮台目标系统
                    turretTargeting.updateCarPosition(pose2dXCM, pose2dYCM);
                    turretTargeting.updateCarHeading(carHeading);

                    if (isTurretAutoAimingActive) {
                        if (fiducialId == 24) {
                            // 检测到fiducial 24时，使用LimeLight的X角度作为炮台当前位置
                            turretCurrentPosition = xDegrees;
                            // 根据距离设置目标角度
                            if (distance >= 260) {
                                turretTargetAngle = TURRET_TARGET_ANGLE_FAR;
                            } else {
                                turretTargetAngle = TURRET_TARGET_ANGLE_NEAR;
                            }
                            // 使用检测到目标时的PID参数
                            turretKp = TURRET_KP_SEE;
                            turretKi = TURRET_KI_SEE;
                            turretKd = TURRET_KD_SEE;
                            turretKiMax = TURRET_KI_MAX_SEE;
                            turretMinErrorSetZero = TURRET_MIN_ERROR_SET_ZERO_SEE;
                        } else {
                            // 未检测到fiducial 24时，通过吸球电机位置计算炮台当前位置
                            turretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (MAX_TURRET_DEAD_WHEEL_POSITION / MAX_TURRET_ANGLE);

                            if (fiducialId == 0 && distanceCarToRed <= 48) {
                                turretTargetAngle = 0;
                            } else {
                                turretTargetAngle = -turretTargeting.getTurretTargetAngle();
                            }
                            // 使用未检测到目标时的PID参数
                            turretKp = TURRET_KP_NOT_SEE;
                            turretKi = TURRET_KI_NOT_SEE;
                            turretKd = TURRET_KD_NOT_SEE;
                            turretKiMax = TURRET_KI_MAX_NOT_SEE;
                            turretMinErrorSetZero = TURRET_MIN_ERROR_SET_ZERO_NOT_SEE;
                        }
                    } else {
                        turretTargetAngle = 0;
                        turretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (MAX_TURRET_DEAD_WHEEL_POSITION / MAX_TURRET_ANGLE);
                    }

                    // PID控制算法
                    error = turretTargetAngle - turretCurrentPosition;
                    integral += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= turretMinErrorSetZero) integral = 0;
                    integral = Math.max(-turretKiMax, Math.min(integral, turretKiMax));
                    derivative = (error - previousError) / dt;

                    // 计算炮台功率
                    if (isTurretAutoAimingActive) {
                        if (fiducialId == 24) {
                            turretPower = 0.5 + (error * turretKp + integral * turretKi + derivative * turretKd);
                        } else {
                            turretPower = 0.5 - (error * turretKp + integral * turretKi + derivative * turretKd);
                        }
                    } else {
                        turretPower = 0.5 - (error * turretKp + integral * turretKi + derivative * turretKd);
                    }

                    // 限制功率边界
                    if (Math.abs(error) <= turretMinErrorSetZero) {
                        if (turretPower > 0.5) {
                            turretPower = 0.5 + TURRET_MIN_POWER;
                        }
                        if (turretPower < 0.5) {
                            turretPower = 0.5 - TURRET_MIN_POWER;
                        }
                    } else {
                        if (0 <= turretPower && turretPower < 0.5 - TURRET_MIN_POWER) {
                            turretPower = Math.min(turretPower, 0.5 - TURRET_MIN_POWER);
                        } else if (0.5 + TURRET_MIN_POWER < turretPower && turretPower <= 1) {
                            turretPower = Math.max(0.5 + TURRET_MIN_POWER, turretPower);
                        }
                    }

                    robot.leftTurretServo.setPosition(turretPower);
                    robot.rightTurretServo.setPosition(turretPower);

                    // 确保周期稳定
                    long sleepTime = Math.max(0, (long) ((loopPeriod - (loopTimer.seconds())) * 1000));
                    if (sleepTime > 0) {
                        sleep(sleepTime);
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(e);
            }
        }
    }
}
