package org.firstinspires.ftc.teamcode.FTC_27650_AUTO_NEW;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC_27650_TeleOp_New.TurretTargetingBlue;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;


@TeleOp(name = "BLUE远点_不开闸_不排_3", group = "LinearOpmode")
//@Autonomous(name = "BLUE远点_不开闸_不排3", group = "LinearOpmode")
@Config
//@Disabled
public class BLUE_YUAN_NOT_OPEN extends LinearOpMode {

    /// 飞轮
    public static double flyWheel_kp = 0.0001, flyWheel_ki = 0.000005, flyWheel_kd = 0.00002;
    public static double flyWheel_max = 600, flyWheel_kf = 1, flyWheel_kv = 0;
    public static volatile double flyWheelTargetVelocity = 0;
    public volatile static double xiMotorMinPower = 0;//0.35;
    /// 自动射时间参数
    public static int colorDelayTime = 200;
    public static int timeRotate1 = 350;
    public static int timeRotate2 = 350;
    public static int timeRotate3 = 350;
    public static int strikeUpTime = 300;
    public static int strikeDownTime = 300;
    public static volatile double TurretTargetAngle = 0;//炮台目标角度
    public static volatile double TurretTargetAngle_yuan = 5;//-3.5 往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_1 = 2;//-3    往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_2 = 0;//-3    往右为负，往左为正
    public static volatile double TurretTargetAngle_jin_3 = 0;//-3    往右为负，往左为正
    ///  >=220cm
    public static double TurretKpSeeYuan = 0.003, TurretKiSeeYuan = 0.003, TurretKdSeeYuan = 0.00005,
            TurretkiMaxSeeYuan = 20, TurretKVSeeYuan = 0.0000, TurrentMinErrorSetZeroSeeYuan = 2.5;
    public static double TurretMinPower = 0, TurretTime = 100;
    private final ElapsedTime runtime = new ElapsedTime();
    otherThread otherThread = new otherThread();
    MyRobotHardware_27650_AUTO_new robot = new MyRobotHardware_27650_AUTO_new(this);
    setFlyWheelPowerThread setFlyWheelPowerThread = new setFlyWheelPowerThread();
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    limeLightThread limeLightThread = new limeLightThread();
    // 初始化炮台朝向控制类
    TurretTargetingBlue TurretTargeting = new TurretTargetingBlue();
    pose2d_setTurrentPosition_Thread pose2d_setTurrentPosition_Thread = new pose2d_setTurrentPosition_Thread();
    volatile boolean setRotateMotorPositionThread_using = true;
    /// 弹舱
    volatile int rotateMotorMaxErrorPosition = 0;//100;
    volatile int errorPosition = 1422;//1422
    double rotate_kp = 0.00035, rotate_ki = 0.000001, rotate_kd = 0.000001, kf = 0;
    double kv = 0.001, ki_max = 10000, kvTime = 300;
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
    volatile String colorFront = "无", colorLeft = "无", colorRight = "无";
    volatile double distanceFront = 0, distanceLeft = 0, distanceRight = 0;
    /// /舵机相关变量////
    volatile double strikerServoDownPosition = 0.54;//0.525
    volatile double strikerServoUpPosition = 0.39;//0.37
    volatile double strikerServoPosition = strikerServoDownPosition;
    volatile double angleServoPosition = 0; //初始化位置
    double angleServoSpeed = 0.02;
    volatile double leftTurretServoPosition = 0.5, rightTurretServoPosition = 0.5;
    /// 旋吸
    volatile double xiMotorPower = 0;
    volatile double TurretCurrentPosition = 0;//炮台位置 left:6350
    /// 状态变量
    volatile int a = 1, b = 1, c = 0, g = 1, p = 1;
    volatile long lastPressTime = 0;
    volatile boolean ledColor = false;
    volatile boolean stop_turret_angleServo_flySpeed = false;
    /// 车辆相对于场地实时位置
    volatile double pose2dX = 0, pose2dY = 0;
    volatile double pose2dXCM = 0, pose2dYCM = 0;
    volatile double distanc_car_to_red = 0;
    /// limeLight反馈
    volatile double xDegrees = 0, yDegrees = 0, distance = 0;
    volatile int id = 0, id_Zhong = 0;
    /// /炮台
    volatile double CarHeading = 0;//车辆imu实时角度
    volatile double carHeading = 0;//车辆imu实时角度
    /// 炮台pid参数
    double TurretKpNotSee = 0.02, TurretKiNotSee = 0.025, TurretKdNotSee = 0.00005,
            TurretkiMaxNotSee = 200, TurrentMinErrorSetZeroNotSee = 3;
    /// <= 120cm
    double TurretKpSeeJin_1 = 0.005, TurretKiSeeJin_1 = 0.003, TurretKdSeeJin_1 = 0.00005,
            TurretkiMaxSeeJin_1 = 200, TurretKVSeeJin_1 = 0.00025, TurrentMinErrorSetZeroSeeJin_1 = 5;
    ///  120cm-----160cm
    double TurretKpSeeJin_2 = 0.003, TurretKiSeeJin_2 = 0.003, TurretKdSeeJin_2 = 0.000055,
            TurretkiMaxSeeJin_2 = 200, TurretKVSeeJin_2 = 0.0002, TurrentMinErrorSetZeroSeeJin_2 = 4.5;
    ///  160cm------220cm
    double TurretKpSeeJin_3 = 0.003, TurretKiSeeJin_3 = 0.003, TurretKdSeeJin_3 = 0.00005,
            TurretkiMaxSeeJin_3 = 200, TurretKVSeeJin_3 = 0.0005, TurrentMinErrorSetZeroSeeJin_3 = 4;
    volatile double TurretPower = 0;
    double x = 63.13, y = -19.65, initialHeading = 180; // 单位厘米 初始化位置

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

        // 以特定姿势实例化您的 MecanumDrive
        Pose2d initialPose = new Pose2d(63.13, -19.65, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // actionBuilder 从传递给它的驱动器步骤构建
        TrajectoryActionBuilder xi_1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(58.13, -19.65), Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(62.96, -53.64), Math.toRadians(270))
                .waitSeconds(0.2)
                .lineToY(-55.3405, new TranslationalVelConstraint(20))
                .waitSeconds(0.2)
                .lineToY(-52.3405, new TranslationalVelConstraint(20))
                .waitSeconds(0.2)
                .lineToY(-55.3405, new TranslationalVelConstraint(20));
        //.splineTo(new Vector2d(-11,-51.3405), Math.toRadians(270),
        //new TranslationalVelConstraint(20));

        TrajectoryActionBuilder she_1 = drive.actionBuilder(new Pose2d(62.96, -55.3405, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(57.05, -18.90), Math.toRadians(180));

        TrajectoryActionBuilder xi_2 = drive.actionBuilder(new Pose2d(-17.72, -26.43, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(13.5437, -25.0222), Math.toRadians(270))
                .waitSeconds(0.1)
                .lineToY(-59.0222, new TranslationalVelConstraint(20));
        //.waitSeconds(0.1)
        //.splineTo(new Vector2d(14.5437,-59.0222), Math.toRadians(270),
        //new TranslationalVelConstraint(20));

        TrajectoryActionBuilder she_2 = drive.actionBuilder(new Pose2d(13.5437, -59.0222, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(14.5437, -35.43), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(-17.72, -26.43), Math.toRadians(270));

        TrajectoryActionBuilder StopPosition = drive.actionBuilder(new Pose2d(-17.72, -26.43, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(0, -47.2138), Math.toRadians(270));

        TrajectoryActionBuilder xi_3 = drive.actionBuilder(new Pose2d(-17.72, -26.43, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(37.0263, -25.1043), Math.toRadians(270))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(37.0263, -57.1043), Math.toRadians(270),
                        new TranslationalVelConstraint(35));

        TrajectoryActionBuilder she_3 = drive.actionBuilder(new Pose2d(-38.0263, -57.1043, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-17.72, -26.43), Math.toRadians(270));

        robot.imu.resetYaw();
        sleep(1000);
        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();

        // 我们现在处于运行时！我们总是添加以下内容，以便在必要时能够停止机器人。
        if (isStopRequested()) return;
        otherThread.start();
        setFlyWheelPowerThread.start();
        setRotateMotorPositionThread.start();
        limeLightThread.start();
        pose2d_setTurrentPosition_Thread.start();

        //第0次射击
        flyWheelTargetVelocity = 1800;
        TurretTargetAngle = -16.91; // 射击角度
        angleServoPosition = 0.68;//0.7
        rotateMotorTargetPosition += (step - errorPosition);//发射位
        a = 2;
        sleep(1500);
        ziDongShe();

        xiMotorMinPower = 0.75;
        b = 2;
        rotateMotorNewTargetPosition = rotateMotorTargetPosition;
        runBlocking(new SequentialAction(xi_1.build()));
        sleep(1000);

        runBlocking(new SequentialAction(she_1.build()));

        /*
        if(id_Zhong==21){
            rotateMotorTargetPosition += step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
            sleep(700);
        }
        else if(id_Zhong==22){
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
            sleep(700);
        }
        else if(id_Zhong==23){
            rotateMotorTargetPosition -= step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
            sleep(700);
        }
        ziDongShe();
        flyWheelTargetVelocity = 0;
        TurretTargetAngle = 51.5;//后续射击炮台目标角度

        xiMotorMinPower = 0.75;
        b=2;
        rotateMotorNewTargetPosition = rotateMotorTargetPosition;
        runBlocking(new SequentialAction(xi_1.build()));
        sleep(500);

        if(id_Zhong==21){
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }
        else if(id_Zhong==22){
            rotateMotorTargetPosition -= step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }
        else if(id_Zhong==23){
            rotateMotorTargetPosition += step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }

        xiMotorMinPower = -0.75;
        sleep(150);
        xiMotorMinPower = 0;
        flyWheelTargetVelocity = 1370;
        runBlocking(new SequentialAction(she_1.build()));
        xiMotorMinPower = 0;
        ziDongShe();
        flyWheelTargetVelocity = 0;

        xiMotorMinPower = 0.75;
        b=2;
        rotateMotorNewTargetPosition = rotateMotorTargetPosition;
        runBlocking(new SequentialAction(xi_2.build()));
        sleep(500);

        if(id_Zhong==21){
            rotateMotorTargetPosition -= step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }
        else if(id_Zhong==22){
            rotateMotorTargetPosition += step;
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }
        else if(id_Zhong==23){
            rotateMotorTargetPosition += (step - errorPosition);//发射位
            a = 2;
        }

        xiMotorMinPower = -0.75;
        sleep(150);
        xiMotorMinPower = 0;
        flyWheelTargetVelocity = 1370;
        runBlocking(new SequentialAction(she_2.build()));
        xiMotorMinPower = 0;
        ziDongShe();
        flyWheelTargetVelocity = 0;
        TurretTargetAngle = 0; //恢复炮台初始化位置

        runBlocking(new SequentialAction(StopPosition.build()));
        */


        while (opModeIsActive()) {
            sleep(10);
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
        sleep(500);

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
        telemetry.addData("id中间", "%d", id_Zhong);
        telemetry.addData("id", "%d", id);
        telemetry.addData("x 角度偏移", "%f", xDegrees);
        telemetry.addData("y 角度偏移", "%f", yDegrees);
        telemetry.addData("distance", "%fCM", distance);

        telemetry.addData("车辆X坐标 英寸", pose2dX);
        telemetry.addData("车辆Y坐标 英寸", pose2dY);
        //telemetry.addData("车辆X坐标 厘米", pose2dXCM);
        //telemetry.addData("车辆Y坐标 厘米", pose2dYCM);
        telemetry.addData("车辆角度（度）全场", carHeading); // 转成度更直观
        telemetry.addData("车辆角度（度）", CarHeading); // 转成度更直观
        telemetry.addData("车辆与红色距离）", distanc_car_to_red);

        telemetry.addData("炮台实际位置", "%f", TurretCurrentPosition);
        telemetry.addData("炮台目标角度(度)", TurretTargetAngle);
        telemetry.addData("炮台功率", TurretPower);
        telemetry.addData("a,b,c,g,p", "%d ,%d, %d, %d, %d", a, b, c, g, p);
        telemetry.update();
    }

    public void servoControl() {
        /*
        if(stop_turret_angleServo_flySpeed){
            if(id==20 && 48 < distance && distance<=255){
                angleServoPosition =  0.004 * distance - 0.031;//0.003
            }else if((id==0 && distanc_car_to_red<=88) || (id==24 && distance<=48)){
                angleServoPosition = 0;
            } else{
                angleServoPosition = 0.7;
            }
        }else{
            angleServoPosition = 0;
        }*/

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
        if (distanceFront <= 4 || colors.alpha >= 0.02) {//
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
        rotateMotorNewTargetPosition = rotateMotorTargetPosition;

    }

    public class otherThread extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
                    colorSensor();
                    if (b == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorNewTargetPosition) <= 150) {//
                        if (colorFront.equals("有") && c <= 3) {
                            c += 1;
                            if (c >= 3) {
                                c = 3;
                                b = 1;
                            }
                            rotateMotorTargetPosition += step;//(288/3);
                            rotateMotorNewTargetPosition = rotateMotorTargetPosition;
                        }
                    }
                    servoControl();
                    xiMotor();
                    show();
                }
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

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

                    /*
                    //1. 确定目标速度（用于前馈基准）
                    if(stop_turret_angleServo_flySpeed){
                        if(distance <= 160){
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1100);
                        }else if( 160 < distance && distance <= 185){
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1150);
                        }else if( 185 < distance && distance <= 200){
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1200);
                        }else if( 200< distance && distance <= 230){
                            flyWheelTargetVelocity = Math.max(1300, 2.35 * distance + 1250);
                        }
                        else{
                            flyWheelTargetVelocity = 1850;
                        }
                    }else{
                        flyWheelTargetVelocity = 0;
                    }
                     */

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
                    rotateMotorPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd
                            + step * kf + (step / kvTime) * kv;
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
        private static final int VALID_FRAME_THRESHOLD = 3;
        private static final int QUEUE_SIZE = 5; // 队列长度，可根据需求调整
        private double lastValidDistance = 0.0;
        private double lastValidXDegrees = 0.0; // 新增：保存最后有效的xDegrees
        private int validFrameCount = 0;
        // 距离滑动平均队列（已存在）
        private Deque<Double> distanceQueue = new ArrayDeque<>();
        // 新增：xDegrees滑动平均队列
        private Deque<Double> xDegreesQueue = new ArrayDeque<>();

        public void run() {
            double a1 = 27;
            double h1 = 29.25;
            double h2 = 74.1;

            while (opModeIsActive()) {
                // 关键1：每次循环先重置ID为0，避免残留旧值
                //id = 0;

                LLResult result = robot.limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    // 标记是否识别到ID=20（核心ID）
                    boolean isId20Detected = false;
                    // 临时存储ID=20的目标数据
                    double tempRawXDegrees = 0.0;
                    double tempYDegrees = 0.0;
                    double tempCurrentDistance = 0.0;

                    if (!fiducialResults.isEmpty()) {
                        // 遍历所有识别到的AprilTag
                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            int currentId = fr.getFiducialId();

                            // 关键2：先赋值ID（无论是不是20号，确保ID一定会被读取）
                            id = currentId;

                            // 仅对20号ID计算距离/角度
                            if (currentId == 20) {
                                tempRawXDegrees = fr.getTargetXDegrees();
                                tempYDegrees = fr.getTargetYDegrees();
                                tempCurrentDistance = (h2 - h1) / Math.tan((a1 + tempYDegrees) * (Math.PI / 180.0));
                                isId20Detected = true;
                                // 如需只读取第一个识别到的ID，保留break；如需读取最后一个，注释掉break
                                // break;
                            }
                            // 其他ID仅赋值id，不做任何额外计算
                        }
                    }

                    // ========== 20号ID的完整处理逻辑（原逻辑保留） ==========
                    if (isId20Detected) {
                        if (tempCurrentDistance > 0 && tempCurrentDistance < 500
                                && tempRawXDegrees > -30 && tempRawXDegrees < 30) {
                            validFrameCount++;
                            if (validFrameCount >= VALID_FRAME_THRESHOLD) {
                                // 距离滤波
                                distanceQueue.add(tempCurrentDistance);
                                if (distanceQueue.size() > QUEUE_SIZE) {
                                    distanceQueue.pollFirst();
                                }
                                distance = calculateAverage(distanceQueue);
                                lastValidDistance = distance;

                                // x角度滤波
                                xDegreesQueue.add(tempRawXDegrees);
                                if (xDegreesQueue.size() > QUEUE_SIZE) {
                                    xDegreesQueue.pollFirst();
                                }
                                xDegrees = calculateAverage(xDegreesQueue);
                                lastValidXDegrees = xDegrees;

                                yDegrees = tempYDegrees;
                            }
                        } else {
                            validFrameCount = 0;
                        }
                    }
                    // ========== 其他ID的处理逻辑（仅保留ID，重置其他值） ==========
                    else if (id != 0) { // 识别到非20号ID
                        validFrameCount = 0;
                        xDegrees = 0.0;   // 其他ID不计算角度，重置为0
                        yDegrees = 0.0;
                        distance = 0.0;   // 其他ID不计算距离，重置为0
                    }
                    // ========== 未识别到任何ID ==========
                    else {
                        validFrameCount = 0;
                        xDegrees = lastValidXDegrees;
                        yDegrees = 0;
                        distance = lastValidDistance;
                    }
                } else {
                    // 无有效结果时的处理
                    validFrameCount = 0;
                    xDegrees = lastValidXDegrees;
                    yDegrees = 0;
                    distance = lastValidDistance;
                }
            }
            robot.limelight.stop();
        }

        // 计算队列平均值（复用现有方法）
        private double calculateAverage(Deque<Double> queue) {
            if (queue.isEmpty()) {
                return 0.0;
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
                Pose2d initialPose = new Pose2d(x, y, Math.toRadians(initialHeading));
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
                    carHeading = CarHeading;
                    if (carHeading < 0) {
                        carHeading = carHeading + 360;
                    }
                    distanc_car_to_red = Math.sqrt((-150 - pose2dXCM) * (-150.0 - pose2dXCM) + (-141.0 - pose2dYCM) * (-141.0 - pose2dYCM));

                    TurretTargeting.updateCarPosition(pose2dXCM, pose2dYCM);
                    TurretTargeting.updateCarHeading(CarHeading);
                    /*
                    if(stop_turret_angleServo_flySpeed){
                        // 3. 获取炮台目标角
                        if(id==ID && pose2dXCM>=90){
                            if(distance>=260){
                                TurretCurrentPosition = xDegrees;
                                TurretTargetAngle = TurretTargetAngle_yuan;//-7.5
                                TurretKp = TurretKpSeeYuan; TurretKi = TurretKiSeeYuan; TurretKd = TurretKdSeeYuan;TurretKV = TurretKVSeeYuan;
                                TurretkiMax = TurretkiMaxSeeYuan;TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeYuan;
                            }
                        }else if(id==ID && pose2dXCM<90){
                            TurretCurrentPosition = xDegrees;
                            if(pose2dXCM<=-120 ){
                                if(distance<=120){
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                }else if(120<distance && distance<=160){
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                }else{
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            }else if (-120<pose2dXCM && pose2dXCM<=60){
                                if(distance<=120){
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                }else if(120<distance && distance<=160){
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                }else{
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            }
                            else if (-60<pose2dXCM && pose2dXCM<=60){
                                if(distance<=120){
                                    TurretTargetAngle = TurretTargetAngle_jin_1;
                                }else if(120<distance && distance<=160){
                                    TurretTargetAngle = TurretTargetAngle_jin_2;
                                }else{
                                    TurretTargetAngle = TurretTargetAngle_jin_3;
                                }
                            }
                            if(distance<=120){
                                TurretKp = TurretKpSeeJin_1; TurretKi = TurretKiSeeJin_1; TurretKd = TurretKdSeeJin_1;TurretKV = TurretKVSeeJin_1;
                                TurretkiMax = TurretkiMaxSeeJin_1;TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_1;
                            }else if( 120<distance && distance<=160){
                                TurretKp = TurretKpSeeJin_2; TurretKi = TurretKiSeeJin_2; TurretKd = TurretKdSeeJin_2;TurretKV = TurretKVSeeJin_2;
                                TurretkiMax = TurretkiMaxSeeJin_2;TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_2;
                            }
                            else if( 160<distance && distance<=220){
                                TurretKp = TurretKpSeeJin_3; TurretKi = TurretKiSeeJin_3; TurretKd = TurretKdSeeJin_3;TurretKV = TurretKVSeeJin_3;
                                TurretkiMax = TurretkiMaxSeeJin_3;TurrentMinErrorSetZero = TurrentMinErrorSetZeroSeeJin_3;
                            }

                        }else{
                            TurretCurrentPosition = -robot.xiMotor.getCurrentPosition()/(maxTurretDeadWhellPosition/maxTurretAngle);
                            if(id==0 && distanc_car_to_red<=88) {TurretTargetAngle = 0;}
                            else {TurretTargetAngle = -TurretTargeting.getTurretTargetAngle();}

                            TurretKp = TurretKpNotSee; TurretKi = TurretKiNotSee; TurretKd = TurretKdNotSee;
                            TurretkiMax = TurretkiMaxNotSee; TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                        }
                    }else{
                        TurretTargetAngle = 0;
                        TurretCurrentPosition = -robot.xiMotor.getCurrentPosition()/(maxTurretDeadWhellPosition/maxTurretAngle);
                        TurretKp = TurretKpNotSee; TurretKi = TurretKiNotSee; TurretKd = TurretKdNotSee;
                        TurretkiMax = TurretkiMaxNotSee; TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                    }
                     */
                    ///旋转炮台算法
                    TurretCurrentPosition = -robot.xiMotor.getCurrentPosition() / (maxTurretDeadWhellPosition / maxTurretAngle);
                    TurretKp = TurretKpNotSee;
                    TurretKi = TurretKiNotSee;
                    TurretKd = TurretKdNotSee;
                    TurretkiMax = TurretkiMaxNotSee;
                    TurrentMinErrorSetZero = TurrentMinErrorSetZeroNotSee;
                    error = TurretTargetAngle - TurretCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) <= TurrentMinErrorSetZero) ki = 0;
                    ki = Math.max(-TurretkiMax, Math.min(ki, TurretkiMax));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    /*
                    if(stop_turret_angleServo_flySpeed){
                        if(id==ID){
                            TurretPower = 0.5 + (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV*(maxTurretDeadWhellPosition/TurretTime));
                        }else{
                            TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV*(maxTurretDeadWhellPosition/TurretTime));
                        }
                    }else{
                        TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV*(maxTurretDeadWhellPosition/TurretTime));
                    }*/
                    TurretPower = 0.5 - (error * TurretKp + ki * TurretKi + kd * TurretKd + TurretKV * (maxTurretDeadWhellPosition / TurretTime));
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












