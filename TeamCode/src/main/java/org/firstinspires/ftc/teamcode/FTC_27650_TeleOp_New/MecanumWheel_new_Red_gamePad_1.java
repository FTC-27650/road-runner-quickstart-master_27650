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


@TeleOp(name = "手动27650_new_red_手柄1", group = "LinearOpmode")
@Config
//@Disabled
public class MecanumWheel_new_Red_gamePad_1 extends LinearOpMode {

    public static int rotateMotorMaxErrorPosition = 100;//2;
    public static int errorPosition = 1422;//1422
    public static double rotate_kp = 0.00035, rotate_ki = 0.00005, rotate_kd = 0.000008, ki_max = 5000, kf = 0;
    public static double rotateMotorMaxPower = 0.8;
    public static double rotateMotorMinPower = 0.05;
    /// /舵机相关变量////
    public static volatile double strikerServoDownPosition = 0.54;//0.525
    public static volatile double strikerServoUpPosition = 0.37;
    public static volatile double angleServoPosition = 0; //初始化位置
    public static volatile double flyWheelTargetVelocity = 0;
    public volatile static double xiMotorMinPower = 0;//0.35;
    public static int colorDelayTime = 200;
    public static int timeRotate = 450;
    public static int strikeUpTime = 250;
    public static int strikeDownTime = 250;
    public static double turretTargetAngle_yuan = -7.5;//-7.5
    public static double turretTargetAngle_jin = -3;//-7.5
    public static double TurretMinPower = 0;
    final float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    final float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    final float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    private final ElapsedTime runtime = new ElapsedTime();
    MecanumThread MecanumThread = new MecanumThread();
    otherThread otherThread = new otherThread();
    MyRobotHardware_27650_TeleOp_new robot = new MyRobotHardware_27650_TeleOp_new(this);
    ledControlThread ledControlThread = new ledControlThread();
    setFlyWheelVelocityThread setFlyWheelVelocityThread = new setFlyWheelVelocityThread();
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    limeLightThread limeLightThread = new limeLightThread();
    // 初始化炮台朝向控制类
    TurretTargetingRed TurretTargeting = new TurretTargetingRed();
    pose2d_setTurrentPosition_Thread pose2d_setTurrentPosition_Thread = new pose2d_setTurrentPosition_Thread();
    volatile boolean setRotateMotorPositionThread_using = true;
    long rotateMotorCurrentPosition = 0;
    long rotateMotorOldTargetPosition = 0;
    long rotateMotorTargetPosition = 0;
    double rotateMotorPower = 0;
    int step = 2731; //96;
    double greenMin = 140, greenMax = 180;
    double purpleMin = 215, purpleMax = 260;
    float gain = 3;//颜色传感器增益值，要>=1
    volatile String colorFront = "无", colorLeft = "无", colorRight = "无";
    volatile double distanceFront = 0, distanceLeft = 0, distanceRight = 0;
    volatile double strikerServoPosition = strikerServoDownPosition;
    double angleServoSpeed = 0.02;
    volatile double leftTurretServoPosition = 0.5, rightTurretServoPosition = 0.5;
    double flyWheel_kp = 7, flyWheel_ki = 1.5, flyWheel_kd = 0.05;
    double flyWheel_max = 600, flyWheel_kf = 1;
    volatile double flyWheelCurrentVelocity = 0;
    volatile double flyWheelMaxVelocity = 2500;
    volatile double xiMotorPower = 0;
    volatile double TurretCurrentPosition = 0;//炮台位置 left:6350
    volatile int a = 1, b = 1, c = 0, g = 1, p = 1;
    volatile long lastPressTime = 0;
    volatile boolean ledColor = false;
    volatile boolean stop_turret_angleServo_flySpeed = false;
    //车辆实时位置
    volatile double pose2dX = 0, pose2dY = 0;
    volatile double pose2dXCM = 0, pose2dYCM = 0;
    volatile double distanc_car_to_red = 0;
    //limeLight反馈
    volatile double xDegrees = 0, yDegrees = 0, distance = 0;
    volatile int id = 0;
    //炮台
    volatile double CarHeading = 0;//车辆imu实时角度
    volatile double turretTargetAngle = 0;//炮台目标角度
    double TurretKp = 0, TurretKi = 0, TurretKd = 0, TurretkiMax = 0, TurrentMinErrorSetZero = 0;
    double TurretKpNotSee = 0.02, TurretKiNotSee = 0.025, TurretKdNotSee = 0.00005,
            TurretkiMaxNotSee = 20, TurrentMinErrorSetZeroNotSee = 3;
    double TurretKpSee = 0.003, TurretKiSee = 0.003, TurretKdSee = 0.00005,
            TurretkiMaxSee = 20, TurrentMinErrorSetZeroSee = 3;
    volatile double TurretPower = 0;

    // 以特定姿势实例化您的 MecanumDrive
    Pose2d initialPose = new Pose2d(60, -167.5, Math.toRadians(90));

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.setMsTransmissionInterval(11);
        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();
        MecanumInit();

        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();

        MecanumThread.start();
        otherThread.start();
        ledControlThread.start();
        setFlyWheelVelocityThread.start();
        setRotateMotorPositionThread.start();
        limeLightThread.start();
        pose2d_setTurrentPosition_Thread.start();

        while (opModeIsActive()) {
            buttonControlRotateMotor();
        }
    }

    public void MecanumInit() {
        setRotateMotorPositionThread_using = false;
        robot.rotateMotor.setPower(0);
        strikerServoPosition = strikerServoDownPosition;
        robot.strikerServo.setPosition(strikerServoPosition);
        sleep(200);
        runtime.reset();
        while (runtime.seconds() <= 0.2) {
            robot.rotateMotor.setPower(-0.3);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);
        strikerServoPosition = strikerServoDownPosition;
        sleep(200);
        robot.strikerServo.setPosition(strikerServoDownPosition);
        robot.angleServo.setPosition(angleServoPosition);
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
        rotateMotorOldTargetPosition = 0;
        gain = 2;
        robot.rotateMotorEncoderRest();
        a = 1;
        c = 0;
        b = 1;
        g = 1;
        p = 1;
        ledColor = false;
        sleep(300);
        setRotateMotorPositionThread_using = true;
        setRotateMotorPositionThread.start();
    }

    public void show() {
        telemetry.clear();
        telemetry.addData("imu", "%4.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //telemetry.addData("向上抬球", "%4.2f", strikerServoPosition);
        telemetry.addData("射球角度", "%4.2f", angleServoPosition);
        //telemetry.addData("Gain", gain);
        //telemetry.addData("色调 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[0], hsvValuesLeft[0], hsvValuesRight[0]);
        //telemetry.addData("饱和度 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[1], hsvValuesLeft[1], hsvValuesRight[1]);
        //telemetry.addData("距离 前/左/右", "%.3f,%.3f, %.3f", distanceFront, distanceLeft, distanceRight);
        //telemetry.addData("Value 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[2], hsvValuesLeft[2], hsvValuesRight[2]);
        //telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
        //telemetry.addData("磁性限位开关 in", " %b", robot.magnetic_in.isPressed());
        //telemetry.addData("旋转功率", "%4.2f", rotateMotorPower);
        //telemetry.addData("旋转位置", "%7d", rotateMotorCurrentPosition);
        //telemetry.addData("角度", "%.1f°", angleDeg);
        //telemetry.addData("位置", absPos);
        //telemetry.addData("目标位置 old/new/差值", "%7d ,%7d, %7d", rotateMotorOldTargetPosition, rotateMotorTargetPosition, rotateMotorTargetPosition - rotateMotorOldTargetPosition);

        telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), flyWheelCurrentVelocity);
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
        telemetry.addData("炮台实际位置", "%f", TurretCurrentPosition);
        telemetry.addData("炮台目标角度(度)", turretTargetAngle);
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
        if (rotateMotorTargetPosition % step == 0) {
            strikerServoPosition = strikerServoDownPosition;  //一键下降
        }
        if (gamepad1.dpad_up)
            angleServoPosition = Math.min(angleServoPosition + angleServoSpeed, 1);
        if (gamepad1.dpad_down)
            angleServoPosition = Math.max(angleServoPosition - angleServoSpeed, 0);

        if (stop_turret_angleServo_flySpeed) {
            if (id == 24 && distance <= 255) {
                angleServoPosition = 0.004 * distance - 0.031;//0.003
            } else if (id == 0 && distanc_car_to_red <= 48) {
                angleServoPosition = 0;
            } else {
                angleServoPosition = 0.7;
            }
        }
        robot.strikerServo.setPosition(strikerServoPosition);
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

        // 前边颜色传感器
        if (distanceFront <= 3.5) {
            colorFront = "有";
        } else {
            colorFront = "无";

        }
        // 左边颜色传感器
        if (hsvValuesLeft[2] <= 0.01 && hsvValuesLeft[0] <= 130
                && (hsvValuesLeft[1] == 1) || hsvValuesLeft[1] == 0) {
            colorLeft = "无";
        } else {
            if ((greenMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= greenMax)
                    && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95)
                    && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) { // 增加饱和度阈值
                colorLeft = "green";
            } else if ((purpleMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= purpleMax)
                    && (0.15 < hsvValuesLeft[1] && hsvValuesLeft[1] < 0.95)
                    && hsvValuesLeft[1] > 0.01 && distanceLeft <= 2.5) {
                colorLeft = "purple";
            } else {
                colorLeft = "有";
            }
        }
        // 右边颜色传感器
        if (hsvValuesRight[2] <= 0.01 && hsvValuesRight[0] <= 130
                && (hsvValuesRight[1] == 1 || hsvValuesRight[1] == 0)) {
            colorRight = "无";
        } else {
            if ((greenMin <= hsvValuesRight[0] && hsvValuesRight[0] <= greenMax)
                    && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95)
                    && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) { // 增加饱和度阈值
                colorRight = "green";
            } else if ((purpleMin <= hsvValuesRight[0] && hsvValuesRight[0] <= purpleMax)
                    && (0.15 < hsvValuesRight[1] && hsvValuesRight[1] < 0.95)
                    && hsvValuesRight[1] > 0.01 && distanceRight <= 2.5) {
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

    /// 彩灯程序发送单个精确宽度的脉冲
    private void sendShortPulse(int widthMs) {
        robot.ledPin.setState(false);
        sleep(widthMs);
        robot.ledPin.setState(true);
    }

    // 按键控制线程
    public void buttonControlRotateMotor() {
        final int downTime = strikeDownTime;
        // 控制进球转到下一步
        if (gamepad1.left_bumper) {
            while (gamepad1.left_bumper) sleep(10);
            strikerServoPosition = strikerServoDownPosition;
            sleep(downTime);  // 等待击球器下降完成
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
            stop_turret_angleServo_flySpeed = false;
            xiMotorMinPower = 0.9;
            strikerServoPosition = strikerServoDownPosition;
            sleep(downTime);
            if (a == 1) rotateMotorTargetPosition += 0;
            if (a == 2) rotateMotorTargetPosition += errorPosition;
            a = 1;
            b = 2;
            g = 1;
            p = 1;
            ledColor = false;
        }
        // 控制射球转到下一步
        if (gamepad1.right_bumper) {
            while (gamepad1.right_bumper) sleep(10);
            strikerServoPosition = strikerServoDownPosition;
            sleep(downTime);
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
            strikerServoPosition = strikerServoDownPosition;
            sleep(downTime);
            if (a == 2 && (colorLeft.equals("green") || colorRight.equals("green"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (a == 2) rotateMotorTargetPosition += step;
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            }
            a = 2;
            b = 1;
            g = 2;
            p = 1;
            ledColor = true;
        }
        // 控制射球直到看到紫色球
        if (gamepad1.dpad_right) {
            while (gamepad1.dpad_right) sleep(10);
            strikerServoPosition = strikerServoDownPosition;
            sleep(downTime);
            if (a == 2 && (colorLeft.equals("purple") || colorRight.equals("purple"))) {
                rotateMotorTargetPosition += 0;
            } else {
                if (a == 2) rotateMotorTargetPosition += step;
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);
            }
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
            sleep(strikeUpTime);
            strikerServoPosition = strikerServoDownPosition;  //一键下降
            sleep(strikeUpTime);
        }
        //手动下降
        if (gamepad1.dpad_down) strikerServoPosition = strikerServoDownPosition;
        //启动自瞄系统
        if (gamepad1.a || c == 3) {
            stop_turret_angleServo_flySpeed = true;
            xiMotorMinPower = 0;
        }

        // 一键转盘初始化
        if (gamepad1.x) {
            while (gamepad1.x) sleep(10);
            MecanumInit();
        }
    }

    public void ziDongShe() {
        if (a == 1) {
            rotateMotorTargetPosition += (step - errorPosition);//(288/3);
            sleep(200);
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = strikerServoUpPosition;  //一键抬升
                sleep(strikeUpTime);
                strikerServoPosition = strikerServoDownPosition;  //一键下降
                sleep(strikeDownTime);
                rotateMotorTargetPosition += step;//(288/3);
                sleep(timeRotate);
            }
        }
        if (a == 2) {
            for (int i = 1; i <= 3; i++) {
                strikerServoPosition = strikerServoUpPosition;  //一键抬升
                sleep(strikeUpTime);
                strikerServoPosition = strikerServoDownPosition;  //一键下降
                sleep(strikeDownTime);
                rotateMotorTargetPosition += step;//(288/3);
                sleep(timeRotate);
            }
        }
        rotateMotorTargetPosition += errorPosition;//(288/3);
        a = 1;
        b = 1;
        g = 1;
        p = 1;
        c = 0;

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
            while (opModeIsActive()) {
                servoControl();
                colorSensor();
                xiMotor();
                show();
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
    public class setFlyWheelVelocityThread extends Thread {
        public void run() {
            try {
                double error;
                double old_error = 0;
                double kd;
                double ki = 0;
                double flyWheelVelocity = 0;
                // 在setRotateMotorPositionThread中使用固定周期控制
                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期
                while (opModeIsActive()) {

                    double dt = loopTimer.seconds();
                    loopTimer.reset();
                    if (stop_turret_angleServo_flySpeed) {
                        if (distance <= 255) {
                            // 原公式基础上增加下限限制，确保速度≥1200
                            flyWheelTargetVelocity = Math.max(1200, 2.35 * distance + 1100);//1048
                        } else {
                            flyWheelTargetVelocity = 1800;
                        }
                    } else {
                        flyWheelTargetVelocity = 900;
                    }

                    flyWheelCurrentVelocity = robot.flyWheelLeft.getVelocity();

                    error = flyWheelTargetVelocity - flyWheelCurrentVelocity;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= 25) ki = 0;
                    ki = Math.max(-flyWheel_max, Math.min(ki, flyWheel_max));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;

                    flyWheelVelocity = error * flyWheel_kp + ki * flyWheel_ki + kd * flyWheel_kd
                            + flyWheelTargetVelocity * flyWheel_kf;

                    //flyWheelVelocity = flyWheelVelocity*(12/robot.batterySensor.getVoltage());

                    robot.flyWheelLeft.setVelocity(flyWheelVelocity + (-gamepad2.right_stick_y * flyWheelMaxVelocity));
                    robot.flyWheelRight.setVelocity(flyWheelVelocity + (-gamepad2.right_stick_y * flyWheelMaxVelocity));
                    old_error = error;

                    sleep(Math.max(0, 10 - (long) (dt * 1000)));  // 确保总周期约10ms
                    // 确保周期稳定（补全不足的时间）
                    double elapsed = loopTimer.seconds();
                    if (elapsed < loopPeriod) {
                        sleep((long) ((loopPeriod - elapsed) * 1000));
                    }
                }
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
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
                    rotateMotorPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + rotateMotorTargetPosition * kf;
                    rotateMotorPower = rotateMotorPower * (12 / robot.batterySensor.getVoltage());
                    if (rotateMotorPower >= rotateMotorMaxPower)
                        rotateMotorPower = rotateMotorMaxPower;
                    else if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower)
                        rotateMotorPower = rotateMotorMinPower;
                    else if (-rotateMotorMinPower <= rotateMotorPower && rotateMotorPower < 0)
                        rotateMotorPower = -rotateMotorMinPower;
                    else if (rotateMotorPower <= -rotateMotorMaxPower)
                        rotateMotorPower = -rotateMotorMaxPower;

                    if (b == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(100);
                        if (colorFront.equals("有") && c < 3) {
                            c += 1;
                            rotateMotorTargetPosition += step;//(288/3);
                            if (c == 3) {
                                b = 1;
                                rotateMotorTargetPosition += (step - errorPosition);//(288/3);
                                a = 2;
                            }
                        }
                    } else if (g == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(colorDelayTime);
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            g = 1;
                        } else {
                            rotateMotorTargetPosition += step;//(288/3);
                        }
                    } else if (p == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(colorDelayTime);
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            p = 1;
                        } else {
                            rotateMotorTargetPosition += step;//(288/3);
                        }
                    }
                    if (Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                    }
                    robot.rotateMotor.setPower(rotateMotorPower);
                    old_error = error;

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

    public class limeLightThread extends Thread {
        public void run() {
            // 您的limelight从完全垂直方向向后旋转了多少度？
            double a1 = 28;//(90-28)=62
            // Limelight镜头中心到地面的距离
            double h1 = 29.5;
            // 目标到地面的距离
            double h2 = 74.1;
            while (opModeIsActive()) {
                LLResult result = robot.limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        id = fr.getFiducialId();
                        xDegrees = fr.getTargetXDegrees();
                        yDegrees = fr.getTargetYDegrees();
                        //计算距离
                        distance = (h2 - h1) / Math.tan((a1 + yDegrees) * (3.14159 / 180.0));
                        //telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                } else {
                    id = 0;
                    xDegrees = 0;
                    yDegrees = 0;
                    distance = 0;
                    //telemetry.addData("Limelight", "No data available");
                }
                //telemetry.update();
            }
            robot.limelight.stop();
        }
    }

    public class pose2d_setTurrentPosition_Thread extends Thread {
        double maxTurretDeadWhellPosition = 7042;
        double maxTurretAngle = 60;

        public void run() {

            try {
                MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

                double error;
                double old_error = 0;
                double kd;
                double ki = 0;

                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期

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
                    distanc_car_to_red = Math.sqrt((-150.0 - pose2dX) * (-150.0 - pose2dX) + (pose2dYCM - 141.0) * (pose2dYCM - 141.0));

                    TurretTargeting.updateCarPosition(pose2dXCM, pose2dYCM);
                    TurretTargeting.updateCarHeading(CarHeading);
                    if (stop_turret_angleServo_flySpeed) {
                        // 3. 获取炮台目标角
                        if (id == 24) {
                            TurretCurrentPosition = xDegrees;
                            if (distance >= 260) {
                                turretTargetAngle = turretTargetAngle_yuan;//-7.5
                            } else {
                                turretTargetAngle = turretTargetAngle_jin;
                            }
                            TurretKp = TurretKpSee;
                            TurretKi = TurretKiSee;
                            TurretKd = TurretKdSee;
                            TurretkiMax = TurretkiMaxSee;
                            TurrentMinErrorSetZero = TurrentMinErrorSetZeroSee;
                        } else {
                            TurretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (maxTurretDeadWhellPosition / maxTurretAngle);

                            if (id == 0 && distanc_car_to_red <= 48) {
                                turretTargetAngle = 0;
                            } else {
                                turretTargetAngle = -TurretTargeting.getTurretTargetAngle();
                            }
                            TurretKp = TurretKpNotSee;
                            TurretKi = TurretKiNotSee;
                            TurretKd = TurretKdNotSee;
                            TurretkiMax = TurretkiMaxNotSee;
                            TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                        }
                    } else {
                        turretTargetAngle = 0;
                        TurretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (maxTurretDeadWhellPosition / maxTurretAngle);
                    }
                    ///旋转炮台算法
                    error = turretTargetAngle - TurretCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= TurrentMinErrorSetZero) ki = 0;
                    ki = Math.max(-TurretkiMax, Math.min(ki, TurretkiMax));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    if (stop_turret_angleServo_flySpeed) {
                        if (id == 24) {
                            TurretPower = 0.5 + (error * TurretKp + ki * TurretKi + kd * TurretKd);
                        } else {
                            TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd);
                        }
                    } else {
                        TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd);
                    }

                    if (Math.abs(error) <= TurrentMinErrorSetZero) {
                        if (TurretPower > 0.5) {
                            TurretPower = 0.5 + TurretMinPower;
                        }
                        if (TurretPower < 0.5) {
                            TurretPower = 0.5 - TurretMinPower;
                        }
                    } else {
                        if (0 <= TurretPower && TurretPower < 0.5 - TurretMinPower) {
                            TurretPower = Math.min(TurretPower, 0.5 - TurretMinPower);
                        } else if (0.5 + TurretMinPower < TurretPower && TurretPower <= 1) {
                            TurretPower = Math.max(0.5 + TurretMinPower, TurretPower);
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












