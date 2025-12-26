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

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;


@TeleOp(name = "手动27650_new_blue_远近皆可-单人", group = "LinearOpmode")
@Config
//@Disabled
public class MecanumWheel_new_blue_gamePad_2_1 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    MecanumThread MecanumThread = new MecanumThread();
    otherThread otherThread = new otherThread();
    MyRobotHardware_27650_TeleOp_new robot = new MyRobotHardware_27650_TeleOp_new(this);
    ledControlThread ledControlThread = new ledControlThread();
    setFlyWheelPowerThread setFlyWheelPowerThread = new setFlyWheelPowerThread();
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    limeLightThread limeLightThread = new limeLightThread();
    // 初始化炮台朝向控制类
    TurretTargetingBlue TurretTargeting = new TurretTargetingBlue();
    pose2d_setTurrentPosition_Thread pose2d_setTurrentPosition_Thread = new pose2d_setTurrentPosition_Thread();

    volatile boolean setRotateMotorPositionThread_using = true;
    /// 飞轮
    public static double flyWheel_kp = 0.0001, flyWheel_ki = 0.000005, flyWheel_kd = 0.00002;
    volatile int errorPosition = 1422;//1422
    public static double flyWheel_max = 600, flyWheel_kf = 1, flyWheel_kv = 0;
    ///  >=220cm
    public static double TurretKpSeeYuan = 0.003, TurretKiSeeYuan = 0.003, TurretKdSeeYuan = 0.00005, TurretkiMaxSeeYuan = 20, TurretKVSeeYuan = 0.0000, TurrentMinErrorSetZeroSeeYuan = 2.5;
    volatile long rotateMotorCurrentPosition = 0;
    volatile long rotateMotorTargetPosition = 0;
    volatile long rotateMotorNewTargetPosition = 0;
    volatile double rotateMotorPower = 0;
    volatile double rotateMotorMaxPower = 1;
    volatile double rotateMotorMinPower = 0;
    volatile double step = 2731; //96;
    /// 颜色传感器
    volatile double greenMin = 140, greenMax = 180;
    volatile double purpleMin = 215, purpleMax = 260;
    volatile float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    volatile float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    volatile float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    volatile float gain = 3;//颜色传感器增益值，要>=1
    /// 弹舱
    volatile int rotateMotorMaxErrorPosition = 0;//100;
    double rotate_kp = 0.00035, rotate_ki = 0.000001, rotate_kd = 0.000001, kf = 0;
    double kv = 0.001, ki_max = 10000, kvTime = 300;
    volatile double strikerServoUpPosition = 0.39;//0.37
    volatile double strikerServoPosition = strikerServoDownPosition;
    volatile double angleServoPosition = 0; //初始化位置
    double angleServoSpeed = 0.02;
    volatile double leftTurretServoPosition = 0.5, rightTurretServoPosition = 0.5;
    volatile String colorFront = "无", colorLeft = "无", colorRight = "无";
    volatile double distanceFront = 0, distanceLeft = 0, distanceRight = 0;
    public static volatile double flyWheelTargetVelocity = 0;
    /// /舵机相关变量////
    volatile double strikerServoDownPosition = 0.54;//0.525
    public volatile static double xiMotorMinPower = 0;//0.35;
    volatile double TurretCurrentPosition = 0;//炮台位置 left:6350
    /// 旋吸
    volatile double xiMotorPower = 0;
    volatile long lastPressTime = 0;
    volatile boolean ledColor = false;
    volatile boolean stop_turret_angleServo_flySpeed = false;
    /// 自动射时间参数
    public static int colorDelayTime = 200;
    public static int timeRotate1 = 350;
    public static int timeRotate2 = 350;
    public static int timeRotate3 = 350;
    public static int strikeUpTime = 300;
    public static int strikeDownTime = 300;
    /// 状态变量
    volatile int a = 1, b = 1, c = 0, g = 1, p = 1;
    volatile double pose2dXCM = 0, pose2dYCM = 0;
    volatile double distanc_car_to_red = 0;
    /// 车辆相对于场地实时位置
    volatile double pose2dX = 0, pose2dY = 0;
    volatile int id = 0;
    /// limeLight反馈
    volatile double xDegrees = 0, yDegrees = 0, distance = 0;
    volatile double TurretTargetAngle = 0;//炮台目标角度
    public static volatile double TurretTargetAngle_yuan = 5;//-3.5 往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_1 = 2;//-3    往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_2 = 0;//-3    往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_3 = 0;//-3    往右为负，往左为正
    /// /炮台
    volatile double CarHeading = 0;//车辆imu实时角度
    /// 炮台pid参数
    double TurretKpNotSee = 0.02, TurretKiNotSee = 0.025, TurretKdNotSee = 0.00005, TurretkiMaxNotSee = 200, TurrentMinErrorSetZeroNotSee = 3;
    /// <= 120cm
    double TurretKpSeeJin_1 = 0.005, TurretKiSeeJin_1 = 0.003, TurretKdSeeJin_1 = 0.00005, TurretkiMaxSeeJin_1 = 200, TurretKVSeeJin_1 = 0.00025, TurrentMinErrorSetZeroSeeJin_1 = 5;
    ///  120cm-----160cm
    double TurretKpSeeJin_2 = 0.003, TurretKiSeeJin_2 = 0.003, TurretKdSeeJin_2 = 0.000055, TurretkiMaxSeeJin_2 = 200, TurretKVSeeJin_2 = 0.0002, TurrentMinErrorSetZeroSeeJin_2 = 4.5;
    ///  160cm------220cm
    double TurretKpSeeJin_3 = 0.003, TurretKiSeeJin_3 = 0.003, TurretKdSeeJin_3 = 0.00005, TurretkiMaxSeeJin_3 = 200, TurretKVSeeJin_3 = 0.0005, TurrentMinErrorSetZeroSeeJin_3 = 4;

    public static double TurretMinPower = 0, TurretTime = 100;
    volatile double TurretPower = 0;
    double x = 157.5, y = -60, initialHeading = -90; // 单位厘米 初始化位置

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.setMsTransmissionInterval(11);
        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();
        strikerServoPosition = strikerServoDownPosition;
        robot.strikerServo.setPosition(strikerServoDownPosition);
        robot.angleServo.setPosition(angleServoPosition);
        robot.leftTurretServo.setPosition(0.5);
        robot.rightTurretServo.setPosition(0.5);
        sleep(200);
        MecanumInit();

        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();

        MecanumThread.start();
        otherThread.start();
        ledControlThread.start();
        setFlyWheelPowerThread.start();
        setRotateMotorPositionThread.start();
        limeLightThread.start();
        pose2d_setTurrentPosition_Thread.start();

        while (opModeIsActive()) {
            buttonControlRotateMotor();
        }
    }

    public void MecanumInit() {

        runtime.reset();
        while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 2) {
            robot.rotateMotor.setPower(-0.5);
            strikerServoPosition = strikerServoDownPosition;
            robot.strikerServo.setPosition(strikerServoDownPosition);
        }
        robot.rotateMotor.setPower(0);
        sleep(100);
        strikerServoPosition = strikerServoDownPosition;
        robot.strikerServo.setPosition(strikerServoDownPosition);
        robot.angleServo.setPosition(angleServoPosition);
        robot.leftTurretServo.setPosition(0.5);
        robot.rightTurretServo.setPosition(0.5);
        sleep(200);

        while (robot.magnetic_in.isPressed()) {
            robot.rotateMotor.setPower(0.1);
        }
        robot.rotateMotor.setPower(0);
        sleep(100);

        rotateMotorTargetPosition = 0;
        gain = 2;
        robot.rotateMotorEncoderRest();
        a = 1;
        c = 0;
        b = 1;
        g = 1;
        p = 1;
        ledColor = false;

    }

    public void show() {
        //telemetry.clear();
        telemetry.addData("imu", "%4.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //telemetry.addData("向上抬球", "%4.2f", strikerServoPosition);
        telemetry.addData("射球角度", "%4.2f", angleServoPosition);
        //telemetry.addData("Gain", gain);
        telemetry.addData("色调 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[0], hsvValuesLeft[0], hsvValuesRight[0]);
        telemetry.addData("饱和度 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[1], hsvValuesLeft[1], hsvValuesRight[1]);
        telemetry.addData("距离 前/左/右", "%.3f,%.3f, %.3f", distanceFront, distanceLeft, distanceRight);
        telemetry.addData("Value 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[2], hsvValuesLeft[2], hsvValuesRight[2]);
        telemetry.addData("Alpha", "%.3f", robot.colorSensorFront.getNormalizedColors().alpha);
        //telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
        //telemetry.addData("磁性限位开关 in", " %b", robot.magnetic_in.isPressed());
        telemetry.addData("旋转功率", "%4.2f", rotateMotorPower);
        telemetry.addData("旋转现在位置", "%7d", rotateMotorCurrentPosition);
        //telemetry.addData("角度", "%.1f°", angleDeg);
        //telemetry.addData("位置", absPos);
        telemetry.addData("目标位置 new/差值", "%7d, %7d", rotateMotorTargetPosition, rotateMotorTargetPosition - rotateMotorCurrentPosition);

        telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), robot.flyWheelLeft.getVelocity());
        telemetry.addData("右飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelRight.getPower(), robot.flyWheelRight.getVelocity());
        telemetry.addData("旋吸功率", "%4.2f", xiMotorPower);

        telemetry.addData("id", "%d", id);
        telemetry.addData("x 角度偏移", "%f", xDegrees);
        telemetry.addData("y 角度偏移", "%f", yDegrees);
        telemetry.addData("distance", "%fCM", distance);

        //telemetry.addData("车辆X坐标 英寸", pose2dX);
        //telemetry.addData("车辆Y坐标 英寸", pose2dY);
        telemetry.addData("车辆X坐标 厘米", pose2dXCM);
        telemetry.addData("车辆Y坐标 厘米", pose2dYCM);
        telemetry.addData("车辆角度（度）", CarHeading); // 转成度更直观
        telemetry.addData("车辆与红色距离）", distanc_car_to_red);

        telemetry.addData("炮台实际位置", "%f", TurretCurrentPosition);
        telemetry.addData("炮台目标角度(度)", TurretTargetAngle);
        telemetry.addData("炮台功率", TurretPower);
        telemetry.addData("a,b,c,g,p", "%d ,%d, %d, %d, %d", a, b, c, g, p);
        telemetry.update();
    }

    public void Mecanum() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

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

    public void servoControl() {

        /*
        if(rotateMotorTargetPosition%step == 0){
            strikerServoPosition = strikerServoDownPosition;  //一键下降
        }
        if (gamepad1.dpad_up)
            angleServoPosition = Math.min(angleServoPosition + angleServoSpeed, 1);
        if (gamepad1.dpad_down)
            angleServoPosition = Math.max(angleServoPosition - angleServoSpeed, 0);
         */
        if (stop_turret_angleServo_flySpeed) {
            if (id == 20 && 48 < distance && distance <= 255) {
                angleServoPosition = 0.004 * distance - 0.031;//0.003
            } else if ((id == 0 && distanc_car_to_red <= 88) || (id == 24 && distance <= 48)) {
                angleServoPosition = 0;
            } else {
                angleServoPosition = 0.7;
            }
        } else {
            angleServoPosition = 0;
        }

        robot.angleServo.setPosition(angleServoPosition);
    }

    public void colorSensor() {
        if (gamepad1.a) gain += 0.005F;
        else if (gamepad1.b && gain > 1) gain -= 0.005F;
        robot.colorSensorFront.setGain(gain);
        robot.colorSensorLeft.setGain(gain);
        robot.colorSensorRight.setGain(gain);

        NormalizedRGBA colorsFront = robot.colorSensorFront.getNormalizedColors();
        NormalizedRGBA colorsLeft = robot.colorSensorLeft.getNormalizedColors();
        NormalizedRGBA colorsRight = robot.colorSensorRight.getNormalizedColors();

        Color.colorToHSV(colorsFront.toColor(), hsvValuesFront);
        Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
        Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);

        distanceFront = ((DistanceSensor) robot.colorSensorFront).getDistance(DistanceUnit.CM);
        distanceLeft = ((DistanceSensor) robot.colorSensorLeft).getDistance(DistanceUnit.CM);
        distanceRight = ((DistanceSensor) robot.colorSensorRight).getDistance(DistanceUnit.CM);

        NormalizedRGBA colors = robot.colorSensorFront.getNormalizedColors();

        // 前边颜色传感器
        if (distanceFront <= 3 || colors.alpha > 0.1) {//
            colorFront = "有";
        } else {
            colorFront = "无";
        }
        // 左边颜色传感器
        if (hsvValuesLeft[2] <= 0.01 && hsvValuesLeft[0] <= 130 && (hsvValuesLeft[1] == 1) || hsvValuesLeft[1] == 0) {
            colorLeft = "无";
        } else {
            if ((greenMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= greenMax) && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95) && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) { // 增加饱和度阈值
                colorLeft = "green";
            } else if ((purpleMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= purpleMax) && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95) && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) {
                colorLeft = "purple";
            } else {
                colorLeft = "有";
            }
        }
        // 右边颜色传感器
        if (hsvValuesRight[2] <= 0.01 && hsvValuesRight[0] <= 130 && (hsvValuesRight[1] == 1 || hsvValuesRight[1] == 0)) {
            colorRight = "无";
        } else {
            if ((greenMin <= hsvValuesRight[0] && hsvValuesRight[0] <= greenMax) && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95) && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) { // 增加饱和度阈值
                colorRight = "green";
            } else if ((purpleMin <= hsvValuesRight[0] && hsvValuesRight[0] <= purpleMax) && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95) && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) {
                colorRight = "purple";
            } else {
                colorRight = "有";
            }
        }

    }

    //gamepad2.left_stick_y 控制吸轮
    public void xiMotor() {

        //xiMotorMinPower = xiMotorMinPower*(12/robot.batterySensor.getVoltage());
        xiMotorPower = -gamepad2.left_stick_y + xiMotorMinPower;
        robot.xiMotor.setPower(xiMotorPower);
    }

    // 按键控制线程
    public void buttonControlRotateMotor() {
        final int downTime = strikeDownTime;
        // 控制进球转到下一步
        if (gamepad1.right_trigger > 0.1) {
            while (gamepad1.right_trigger > 0.1) sleep(10);
            //strikerServoPosition = strikerServoDownPosition;
            //sleep(downTime);  // 等待击球器下降完成
            if (a == 1) rotateMotorTargetPosition += step;
            if (a == 2) rotateMotorTargetPosition += errorPosition;
            a = 1;
            b = 1;
            g = 1;
            p = 1;
            ledColor = false;
        }
        // 控制进球装置进入自动旋转模式
        if (gamepad1.left_trigger > 0.1) {
            while (gamepad1.left_trigger > 0.1) sleep(10);
            c = 0;
            stop_turret_angleServo_flySpeed = false;
            xiMotorMinPower = 0.85;
            //strikerServoPosition = strikerServoDownPosition;
            //sleep(downTime);
            if (a == 1) rotateMotorTargetPosition += 0;
            if (a == 2) rotateMotorTargetPosition += errorPosition;
            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
            a = 1;
            b = 2;
            g = 1;
            p = 1;
            ledColor = false;
        }
        // 控制射球转到下一步
        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) sleep(10);
            //strikerServoPosition = strikerServoDownPosition;
            //sleep(downTime);
            if (a == 2) rotateMotorTargetPosition += step;
            if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            a = 2;
            b = 1;
            g = 1;
            p = 1;
            ledColor = true;
        }
        // 控制射球直到看到绿色球
        if (gamepad1.dpad_left) {
            while (gamepad1.dpad_left) sleep(10);
            //strikerServoPosition = strikerServoDownPosition;
            //sleep(downTime);
            if (a == 2 && (colorLeft.equals("green") || colorRight.equals("green"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (a == 2) rotateMotorTargetPosition += step;
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            }
            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
            a = 2;
            b = 1;
            g = 2;
            p = 1;
            ledColor = true;
        }
        // 控制射球直到看到紫色球
        if (gamepad1.dpad_right) {
            while (gamepad1.dpad_right) sleep(10);
            //strikerServoPosition = strikerServoDownPosition;
            //sleep(downTime);
            if (a == 2 && (colorLeft.equals("purple") || colorRight.equals("purple"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (a == 2) rotateMotorTargetPosition += step;
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            }
            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
            a = 2;
            b = 1;
            g = 1;
            p = 2;
            ledColor = true;
        }
        //自动射三球
        if (gamepad1.y) {
            while (gamepad1.y) sleep(10);
            ziDongShe();
        }
        //手动抬自动落
        if (gamepad1.dpad_up) {
            while (gamepad1.dpad_up) {
                sleep(10);
            }
            strikerServoPosition = strikerServoUpPosition;  //一键抬升
            robot.strikerServo.setPosition(strikerServoPosition);
            sleep(strikeUpTime);
            strikerServoPosition = strikerServoDownPosition;  //一键下降
            robot.strikerServo.setPosition(strikerServoPosition);
            sleep(strikeUpTime);
        }
        //防缠绕
        if (gamepad1.dpad_down) {
            xiMotorMinPower = 0;
            sleep(100);
            xiMotorMinPower = -0.6;
            sleep(200);
            xiMotorMinPower = 0;
            c = 0;
        }
        //启动自瞄系统
        if (gamepad1.a || c == 3) {
            stop_turret_angleServo_flySpeed = true;
            xiMotorMinPower = 0;
            //if (a == 2) rotateMotorTargetPosition += 0;
            //if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            //a=2;
        }
        // 一键转盘初始化
        if (gamepad1.x) {
            while (gamepad1.x) sleep(10);
            // 停止旧线程
            setRotateMotorPositionThread_using = false;
            setRotateMotorPositionThread.interrupt();
            try {
                setRotateMotorPositionThread.join(); // 等待旧线程完全退出
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            robot.rotateMotor.setPower(0); // 释放电机

            MecanumInit();

            try {
                setRotateMotorPositionThread = new setRotateMotorPositionThread();
                setRotateMotorPositionThread_using = true;
                setRotateMotorPositionThread.start();
                sleep(100);
            } catch (IllegalThreadStateException e) {
                telemetry.addData("线程启动失败", e.getMessage());
                telemetry.update();
            }
        }
    }

    public void ziDongShe() {
        if (a == 1) {
            rotateMotorTargetPosition += (step - errorPosition);//(288/3);
            sleep(200);
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = strikerServoUpPosition;  //一键抬升
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(strikeUpTime);
                strikerServoPosition = strikerServoDownPosition;  //一键下降
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(strikeDownTime);
                rotateMotorTargetPosition += step;//(288/3);
                if (i == 1) {
                    sleep(timeRotate1);
                } else if (i == 2) {
                    sleep(timeRotate2);
                } else {
                    sleep(timeRotate3);
                }
            }
        }
        if (a == 2) {
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = strikerServoUpPosition;  //一键抬升
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(strikeUpTime);
                strikerServoPosition = strikerServoDownPosition;  //一键下降
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(strikeDownTime);
                rotateMotorTargetPosition += step;//(288/3);
                if (i == 1) {
                    sleep(timeRotate1);
                } else if (i == 2) {
                    sleep(timeRotate2);
                } else {
                    sleep(timeRotate3);
                }
            }
        }
        rotateMotorTargetPosition += errorPosition;//(288/3);
        a = 1;
        b = 1;
        g = 1;
        p = 1;
        c = 0;

    }

    /// 彩灯程序发送单个精确宽度的脉冲
    private void sendShortPulse(int widthMs) {
        robot.ledPin.setState(false);
        sleep(widthMs);
        robot.ledPin.setState(true);
    }

    public class MecanumThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                Mecanum();
            }
        }
    }

    public class otherThread extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
                    colorSensor();
                    if (b == 2 && (double) rotateMotorTargetPosition % step == 0 && Math.abs(rotateMotorCurrentPosition - rotateMotorNewTargetPosition) <= 150) {//
                        if (colorFront.equals("有") && c <= 3) {
                            c += 1;
                            if (c >= 3) {
                                c = 3;
                                b = 1;
                            }
                            rotateMotorTargetPosition += step;//(288/3);
                            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
                        }
                    } else if (g == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorNewTargetPosition) <= 150) {
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            g = 1;
                        } else {
                            rotateMotorTargetPosition += step;//(288/3);
                            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
                        }
                    } else if (p == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorNewTargetPosition) <= 150) {
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            p = 1;
                        } else {
                            rotateMotorTargetPosition += step;//(288/3);
                            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
                        }
                    }
                    servoControl();
                    xiMotor();
                    show();
                    sleep(10);
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public class ledControlThread extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
                    if ((colorLeft.equals("green") || colorRight.equals("green")) && ledColor) {
                        sendShortPulse(60); // 45ms中脉冲 = 绿
                        lastPressTime = System.currentTimeMillis();
                    } else if ((colorLeft.equals("purple") || colorRight.equals("purple")) && ledColor) {
                        sendShortPulse(40); // 20ms短脉冲 = 红
                        lastPressTime = System.currentTimeMillis();
                    } else if (c == 3) {
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
                throw new RuntimeException(e);
            }
        }
    }

    /*弹舱绝对值位置获取
    public void rotateAbsolutePosition(){
        // 3. 读取数据
        int rawCount = robot.rotateMotor.getCurrentPosition();
        boolean indexState = robot.indexPin.getState(); // true=高电平, false=低电平

        // 4. 索引下降沿触发归零（假设索引低电平有效）
        if (!indexState && lastIndexState) {
            zeroOffset = -rawCount;  // 计算偏移量
        }
        lastIndexState = indexState;

        // 5. 计算绝对角度（0-360度）
        absPos = rawCount + zeroOffset;
        angleDeg = (absPos % COUNTS_PER_REV) * 360.0 / COUNTS_PER_REV;
        if (angleDeg < 0) angleDeg += 360;
    }
     */
    //飞轮线程 gamepad2.right_stick_y 控制飞轮
    public class setFlyWheelPowerThread extends Thread {
        public void run() {
            try {
                // 基础功率（前馈部分，根据目标速度需求设置）
                double basePower = 0.6;  // 基础功率（前馈）
                double adjustPower = 0;  // 反馈调节量

                // 轻量化PID参数（仅用于修正补偿小波动）
                final double kp = flyWheel_kp;//0.0001;   // 比例系数（比纯反馈时小）
                final double ki = flyWheel_ki;//0.000005; // 积分系数（弱化积分，避免超调）
                final double kd = flyWheel_kd;//0.00002;  // 微分系数（抑制突变）
                final double maxAdjust = 0.15; // 限制调节幅度

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01;

                while (opModeIsActive()) {
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    //1. 确定目标速度（用于前馈基准）
                    if (stop_turret_angleServo_flySpeed) {
                        if (distance <= 160) {
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1100);
                        } else if (160 < distance && distance <= 185) {
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1150);
                        } else if (185 < distance && distance <= 200) {
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1200);
                        } else if (200 < distance && distance <= 230) {
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1250);
                        } else {
                            flyWheelTargetVelocity = 1850;
                        }
                    } else {
                        flyWheelTargetVelocity = 0;
                    }

                    // 2. 前馈功率：根据目标速度设置基础功率（线性映射）
                    // 假设最大速度2500对应功率1.0，计算目标速度对应的基础功率
                    basePower = Math.max(0, flyWheelTargetVelocity / 2500.0);

                    // 3. 反馈调节：修正实际速度与目标的偏差
                    double currentVelocity = robot.flyWheelLeft.getVelocity();
                    double error = flyWheelTargetVelocity - currentVelocity;

                    // 积分项（限制范围，避免饱和）
                    double integral = 0;
                    integral += error * dt;
                    integral = Math.max(-0.3, Math.min(integral, 0.3));

                    // 微分项
                    double lastError = 0;
                    double derivative = (error - lastError) / dt;
                    lastError = error;

                    // 计算反馈调节量
                    adjustPower = kp * error + ki * integral + kd * derivative;
                    adjustPower = Math.max(-maxAdjust, Math.min(adjustPower, maxAdjust));

                    // 4. 最终功率 = 前馈 + 反馈 + 手柄微调
                    double finalPower = basePower + adjustPower;
                    finalPower += -gamepad2.right_stick_y * 0.15; // 手柄微调幅度减小
                    finalPower = Math.max(0, Math.min(finalPower, 1.0));

                    // 5. 应用功率
                    robot.flyWheelLeft.setPower(finalPower);
                    robot.flyWheelRight.setPower(finalPower);

                    sleep((long) (loopPeriod * 1000));
                }

                // 停止时关闭功率
                robot.flyWheelLeft.setPower(0);
                robot.flyWheelRight.setPower(0);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public class setRotateMotorPositionThread extends Thread {
        public void run() {
            try {
                double error;
                double old_error = 0;
                double kd;
                double ki = 0;
                // 在setRotateMotorPositionThread中使用固定周期控制
                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期
                while (opModeIsActive() && setRotateMotorPositionThread_using) {

                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();

                    error = rotateMotorTargetPosition - rotateMotorCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= rotateMotorMaxErrorPosition) ki = 0;
                    ki = Math.max(-ki_max, Math.min(ki, ki_max));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    rotateMotorPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + step * kf + (step / kvTime) * kv;
                    //rotateMotorPower = rotateMotorPower*(12/robot.batterySensor.getVoltage());

                    if (rotateMotorPower >= rotateMotorMaxPower)
                        rotateMotorPower = rotateMotorMaxPower;
                    else if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower)
                        rotateMotorPower = rotateMotorMinPower;
                    else if (-rotateMotorMinPower <= rotateMotorPower && rotateMotorPower < 0)
                        rotateMotorPower = -rotateMotorMinPower;
                    else if (rotateMotorPower <= -rotateMotorMaxPower)
                        rotateMotorPower = -rotateMotorMaxPower;

                    robot.rotateMotor.setPower(rotateMotorPower);
                    old_error = error;

                    sleep(Math.max(0, 10 - (long) (dt * 1000)));  // 确保总周期约10ms
                    // 确保周期稳定（补全不足的时间）
                    double elapsed = loopTimer.seconds();
                    if (elapsed < loopPeriod) {
                        sleep((long) ((loopPeriod - elapsed) * 1000));
                    }
                }
                rotateMotorPower = 0;
                robot.rotateMotor.setPower(rotateMotorPower);
            } catch (InterruptedException e) {
                // 中断时释放资源
                robot.rotateMotor.setPower(0);
                Thread.currentThread().interrupt(); // 保留中断状态
            } finally {
                // 确保电机停止
                robot.rotateMotor.setPower(0);
            }
        }
    }

    public class limeLightThread extends Thread {
        private double lastValidDistance = 0.0;
        private double lastValidXDegrees = 0.0; // 新增：保存最后有效的xDegrees
        private int validFrameCount = 0;
        private static final int VALID_FRAME_THRESHOLD = 3;

        // 距离滑动平均队列（已存在）
        private Deque<Double> distanceQueue = new ArrayDeque<>();
        // 新增：xDegrees滑动平均队列
        private Deque<Double> xDegreesQueue = new ArrayDeque<>();
        private static final int QUEUE_SIZE = 5; // 队列长度，可根据需求调整

        public void run() {
            double a1 = 27;
            double h1 = 29.25;
            double h2 = 74.1;

            while (opModeIsActive()) {
                LLResult result = robot.limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    // 新增：标记是否识别到ID=20
                    boolean isId20Detected = false;
                    // 新增：临时存储ID=24的目标数据
                    double tempRawXDegrees = 0.0;
                    double tempYDegrees = 0.0;
                    double tempCurrentDistance = 0.0;

                    if (!fiducialResults.isEmpty()) {
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            // 核心修改：仅处理ID=20的目标
                            if (fr.getFiducialId() == 20) {
                                id = fr.getFiducialId(); // 仅赋值ID=24
                                tempRawXDegrees = fr.getTargetXDegrees(); // 临时存x角度
                                tempYDegrees = fr.getTargetYDegrees(); // 临时存y角度
                                tempCurrentDistance = (h2 - h1) / Math.tan((a1 + tempYDegrees) * (Math.PI / 180.0)); // 临时存距离
                                isId20Detected = true;
                                break; // 找到20号ID后退出循环，避免多目标干扰
                            }
                        }
                    }

                    // 仅当识别到ID=20时，才执行后续有效性校验和滤波
                    if (isId20Detected) {
                        // 同时校验距离和x角度的有效性
                        if (tempCurrentDistance > 0 && tempCurrentDistance < 500 && tempRawXDegrees > -30 && tempRawXDegrees < 30) { // 假设x角度合理范围为±30度
                            validFrameCount++;
                            if (validFrameCount >= VALID_FRAME_THRESHOLD) {
                                // 距离滤波（已存在）
                                distanceQueue.add(tempCurrentDistance);
                                if (distanceQueue.size() > QUEUE_SIZE) {
                                    distanceQueue.pollFirst();
                                }
                                distance = calculateAverage(distanceQueue);
                                lastValidDistance = distance;

                                // 新增：x角度滤波
                                xDegreesQueue.add(tempRawXDegrees);
                                if (xDegreesQueue.size() > QUEUE_SIZE) {
                                    xDegreesQueue.pollFirst();
                                }
                                xDegrees = calculateAverage(xDegreesQueue); // 滤波后的x角度
                                lastValidXDegrees = xDegrees;

                                // 同步更新yDegrees为20号目标的y角度
                                yDegrees = tempYDegrees;
                            }
                        } else {
                            validFrameCount = 0;
                        }
                    } else {
                        // 未识别到ID=20，重置状态并使用最后有效值
                        validFrameCount = 0;
                        id = 0; // 无24号ID时置0
                        xDegrees = lastValidXDegrees;
                        yDegrees = 0; // 无目标时y角度置0
                        distance = lastValidDistance;
                    }
                } else {
                    validFrameCount = 0;
                    id = 0;
                    xDegrees = lastValidXDegrees; // 无有效结果时使用最后有效值
                    yDegrees = 0;
                    distance = lastValidDistance;
                }
            }
            robot.limelight.stop();
        }

        // 计算队列平均值（复用现有方法）
        private double calculateAverage(Deque<Double> queue) {
            if (queue.isEmpty()) {
                return 0.0; // 队列空时返回0，或根据实际需求返回默认值
            }
            double sum = 0.0;
            for (double d : queue) {
                sum += d;
            }
            return sum / queue.size();
        }
    }

    public class pose2d_setTurrentPosition_Thread extends Thread {
        public void run() {
            try {
                // 以特定姿势实例化您的 MecanumDrive
                Pose2d initialPose = new Pose2d(x / 2.54, y / 2.54, Math.toRadians(initialHeading));
                MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
                double maxTurretDeadWhellPosition = 7042;
                double maxTurretAngle = 60;
                double TurretKp = 0, TurretKi = 0, TurretKd = 0, TurretKV = 0, TurretkiMax = 0, TurrentMinErrorSetZero = 0;
                double error;
                double old_error = 0;
                double kd;
                double ki = 0;

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期
                int ID = 20;
                while (opModeIsActive()) {
                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    drive.updatePoseEstimate(); // 先更新
                    // 第二步第二步：获取当前位姿（包含坐标和角度）
                    Pose2d currentPose = drive.localizer.getPose();
                    // 提取x和y坐标（单位：英寸，由代码中的inPerTick决定）
                    pose2dX = currentPose.position.x; // x坐标
                    pose2dY = currentPose.position.y; // y坐标
                    pose2dXCM = currentPose.position.x * 2.54; // x坐标
                    pose2dYCM = currentPose.position.y * 2.54; // y坐标
                    CarHeading = Math.toDegrees(currentPose.heading.toDouble());
                    distanc_car_to_red = Math.sqrt((-150 - pose2dXCM) * (-150.0 - pose2dXCM) + (-141.0 - pose2dYCM) * (-141.0 - pose2dYCM));

                    TurretTargeting.updateCarPosition(pose2dXCM, pose2dYCM);
                    TurretTargeting.updateCarHeading(CarHeading);
                    if (stop_turret_angleServo_flySpeed) {
                        // 3. 获取炮台目标角
                        if (id == ID && pose2dXCM >= 90) {
                            if (distance >= 260) {
                                TurretCurrentPosition = xDegrees;
                                TurretTargetAngle = TurretTargetAngle_yuan;//-7.5
                                TurretKp = TurretKpSeeYuan;
                                TurretKi = TurretKiSeeYuan;
                                TurretKd = TurretKdSeeYuan;
                                TurretKV = TurretKVSeeYuan;
                                TurretkiMax = TurretkiMaxSeeYuan;
                                TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeYuan;
                            }
                        } else if (id == ID && pose2dXCM < 90) {
                            TurretCurrentPosition = xDegrees;
                            if (pose2dXCM <= -120) {
                                if (distance <= 120) {
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                } else if (120 < distance && distance <= 160) {
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                } else {
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            } else if (-120 < pose2dXCM && pose2dXCM <= 60) {
                                if (distance <= 120) {
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                } else if (120 < distance && distance <= 160) {
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                } else {
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            } else if (-60 < pose2dXCM && pose2dXCM <= 60) {
                                if (distance <= 120) {
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                } else if (120 < distance && distance <= 160) {
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                } else {
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            }
                            if (distance <= 120) {
                                TurretKp = TurretKpSeeJin_1;
                                TurretKi = TurretKiSeeJin_1;
                                TurretKd = TurretKdSeeJin_1;
                                TurretKV = TurretKVSeeJin_1;
                                TurretkiMax = TurretkiMaxSeeJin_1;
                                TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_1;
                            } else if (120 < distance && distance <= 160) {
                                TurretKp = TurretKpSeeJin_2;
                                TurretKi = TurretKiSeeJin_2;
                                TurretKd = TurretKdSeeJin_2;
                                TurretKV = TurretKVSeeJin_2;
                                TurretkiMax = TurretkiMaxSeeJin_2;
                                TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_2;
                            } else if (160 < distance && distance <= 220) {
                                TurretKp = TurretKpSeeJin_3;
                                TurretKi = TurretKiSeeJin_3;
                                TurretKd = TurretKdSeeJin_3;
                                TurretKV = TurretKVSeeJin_3;
                                TurretkiMax = TurretkiMaxSeeJin_3;
                                TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_3;
                            }

                        } else {
                            TurretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (maxTurretDeadWhellPosition / maxTurretAngle);
                            if (id == 0 && distanc_car_to_red <= 88) {
                                TurretTargetAngle = 0;
                            } else {
                                TurretTargetAngle = -TurretTargeting.getTurretTargetAngle();
                            }

                            TurretKp = TurretKpNotSee;
                            TurretKi = TurretKiNotSee;
                            TurretKd = TurretKdNotSee;
                            TurretkiMax = TurretkiMaxNotSee;
                            TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                        }
                    } else {
                        TurretTargetAngle = 0;
                        TurretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (maxTurretDeadWhellPosition / maxTurretAngle);
                        TurretKp = TurretKpNotSee;
                        TurretKi = TurretKiNotSee;
                        TurretKd = TurretKdNotSee;
                        TurretkiMax = TurretkiMaxNotSee;
                        TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                    }
                    ///旋转炮台算法
                    error = TurretTargetAngle - TurretCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= TurrentMinErrorSetZero) ki = 0;
                    ki = Math.max(-TurretkiMax, Math.min(ki, TurretkiMax));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    if (stop_turret_angleServo_flySpeed) {
                        if (id == ID) {
                            TurretPower = 0.5 + (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV * (maxTurretDeadWhellPosition / TurretTime));
                        } else {
                            TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV * (maxTurretDeadWhellPosition / TurretTime));
                        }
                    } else {
                        TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV * (maxTurretDeadWhellPosition / TurretTime));
                    }
                    TurretPower = Math.max(0.0, Math.min(1.0, TurretPower));

                    if (Math.abs(error) <= TurrentMinErrorSetZero) {
                        TurretPower = 0.5;
                    } else {
                        if (TurretPower > 0.5) {
                            TurretPower = TurretPower + TurretMinPower;
                        } else if (TurretPower < 0.5) {
                            TurretPower = TurretPower - TurretMinPower;
                        }
                    }

                    robot.leftTurretServo.setPosition(TurretPower);
                    robot.rightTurretServo.setPosition(TurretPower);

                    sleep(Math.max(0, 10 - (long) (dt * 1000)));  // 确保总周期约10ms
                    // 确保周期稳定（补全不足的时间）
                    double elapsed = loopTimer.seconds();
                    if (elapsed < loopPeriod) {
                        sleep((long) ((loopPeriod - elapsed) * 1000));
                    }
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}












