package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "手动27650——new_3", group = "LinearOpmode")
@Config

public class MecanumWheel_new_5 extends LinearOpMode {

    public static double greenMin = 140, greenMax = 195;
    public static double purpleMin = 215, purpleMax = 260;
    public static int rotateMotorMaxErrorPosition = 2;
    public static int errorPosition = 15;
    public static double rotate_kp = 0.02, rotate_ki = 0.001, rotate_kd = 0.002, kf = 0;//0.0001;
    public static double ki_max = 1000;
    public static double flyWheelMaxVelocity = 3500;
    public static double xiMotorMinPower = 0;
    private static int greenFrontCount = 0, purpleFrontCount = 0;
    private static int greenLeftCount = 0, purpleLeftCount = 0;
    private static int greenRightCount = 0, purpleRightCount = 0;
    final float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    final float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    final float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    private final ElapsedTime runtime = new ElapsedTime();
    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    camera cameraThread = new camera();
    double strikerServoDownPosition = 0.55;//角度舵机    0.49代表转到中间
    double strikerServoUpPosition = 0.85;//角度舵机    0.49代表转到中间
    double strikerServoPosition = strikerServoDownPosition;
    double angleServoPosition = 0; //初始化位置
    double angleServoSpeed = 0.01;
    float gain = 3;//颜色传感器增益值，要>=1
    volatile String colorFront = "无";
    volatile String colorLeft = "无";
    volatile String colorRight = "无";
    boolean magnetic_in_bool = false;
    int rotateMotorCurrentPosition = 0;
    int rotateMotorOldTargetPosition = 0;
    int rotateMotorTargetPosition = 0;
    double rotatePowerStart = 0.1;
    double rotateMotorPower = 0;
    double rotateMotorMinPower = 0.1;
    double rotateMotorMaxPower = 0.8;
    volatile double flyWheelTargetVelocity = 0;
    double xiMotorPower = 0;
    volatile int a = 1;
    int b = 1;
    int c = 0;
    volatile int g = 1;
    int p = 1;
    int step = 96;


    @Override
    public void runOpMode() {

        robot.init();
        robot.strikerServo.setPosition(strikerServoPosition);
        robot.angleServo.setPosition(angleServoPosition);
        sleep(200);
        runtime.reset();
        while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 3) {
            robot.rotateMotor.setPower(rotatePowerStart);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);

        rotateMotorTargetPosition = 0;
        rotateMotorOldTargetPosition = 0;
        gain = 3;

        robot.rotateMotorEncoderRest();
        sleep(300);
        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();
        cameraThread.start();
        setRotateMotorPositionThread.start();
        while (opModeIsActive()) {
            servoControl();
            colorSensor();
            Mecanum();
            flyWheelControl();
            xiMotor();
            ledControl();

            //show();
        }
    }


    public void show() {

        magnetic_in_bool = robot.magnetic_in.isPressed();

        telemetry.addData("imu", "%4.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("向上抬球", "%4.2f", strikerServoPosition);
        telemetry.addData("射球角度", "%4.2f", angleServoPosition);
        telemetry.addData("Gain", gain);
        telemetry.addData("色调 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[0], hsvValuesLeft[0], hsvValuesRight[0]);
        telemetry.addData("饱和度 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[1], hsvValuesLeft[1], hsvValuesRight[1]);
        telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
        telemetry.addData("磁性限位开关 in", " %b", magnetic_in_bool);
        telemetry.addData("旋转功率", "%4.2f", rotateMotorPower);
        telemetry.addData("旋转位置", "%7d", rotateMotorCurrentPosition);
        telemetry.addData("目标位置 old/new/差值", "%7d ,%7d, %7d", rotateMotorOldTargetPosition, rotateMotorTargetPosition, rotateMotorTargetPosition - rotateMotorOldTargetPosition);
        telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), robot.flyWheelLeft.getVelocity());
        telemetry.addData("右飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelRight.getPower(), robot.flyWheelRight.getVelocity());
        telemetry.addData("旋吸功率", "%4.2f", xiMotorPower);
        telemetry.addData("a,b,c,kp", "%d ,%d, %d, %4.2f", a, b, c, rotate_kp);
        telemetry.addData("g, p", "%d, %d", g, p);
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")


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
            flPower = flPower * 0.6;
            frPower = frPower * 0.6;
            brPower = brPower * 0.6;
            blPower = blPower * 0.6;

        }

        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.br.setPower(brPower);
        robot.bl.setPower(blPower);
    }

    public void servoControl() {
        if (gamepad2.dpad_up) strikerServoPosition = strikerServoUpPosition;  //一键抬升
        if (gamepad2.dpad_down) strikerServoPosition = strikerServoDownPosition;  //一键下降

        if (gamepad1.dpad_up)
            angleServoPosition = Math.min(angleServoPosition + angleServoSpeed, 1);
        if (gamepad1.dpad_down)
            angleServoPosition = Math.max(angleServoPosition - angleServoSpeed, 0);

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

        // 前边颜色传感器
        if ((greenMin <= hsvValuesFront[0] && hsvValuesFront[0] <= greenMax) && hsvValuesFront[1] > 0.2) { // 增加饱和度阈值
            greenFrontCount++;
            purpleFrontCount = 0;
        } else if ((purpleMin <= hsvValuesFront[0] && hsvValuesFront[0] <= purpleMax) && hsvValuesFront[1] > 0.2) {
            purpleFrontCount++;
            greenFrontCount = 0;
        } else {
            greenFrontCount = 0;
            purpleFrontCount = 0;
        }
        // 连续3次识别一致才确认
        colorFront = (greenFrontCount >= 3) ? "green" : (purpleFrontCount >= 3) ? "purple" : "无";

        // 左边颜色传感器
        if ((greenMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= greenMax) && hsvValuesLeft[1] > 0.2) { // 增加饱和度阈值
            greenLeftCount++;
            purpleLeftCount = 0;
        } else if ((purpleMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= purpleMax) && hsvValuesLeft[1] > 0.2) {
            purpleLeftCount++;
            greenLeftCount = 0;
        } else {
            greenLeftCount = 0;
            purpleLeftCount = 0;
        }
        // 连续3次识别一致才确认
        colorLeft = (greenLeftCount >= 3) ? "green" : (purpleLeftCount >= 3) ? "purple" : "无";

        //右边颜色传感器
        if ((greenMin <= hsvValuesRight[0] && hsvValuesRight[0] <= greenMax) && hsvValuesRight[1] > 0.2) { // 增加饱和度阈值
            greenRightCount++;
            purpleRightCount = 0;
        } else if ((purpleMin <= hsvValuesRight[0] && hsvValuesRight[0] <= purpleMax) && hsvValuesRight[1] > 0.2) {
            purpleRightCount++;
            greenRightCount = 0;
        } else {
            greenRightCount = 0;
            purpleRightCount = 0;
        }
        // 连续3次识别一致才确认
        colorRight = (greenRightCount >= 3) ? "green" : (purpleRightCount >= 3) ? "purple" : "无";
    }

    //飞轮线程 gamepad2.right_stick_y 控制飞轮
    public void flyWheelControl() {

        double flyWheelVelocity = -gamepad2.right_stick_y * flyWheelMaxVelocity + flyWheelTargetVelocity;
        robot.flyWheelLeft.setVelocity(flyWheelVelocity);
        robot.flyWheelRight.setVelocity(flyWheelVelocity);
    }

    //gamepad2.left_stick_y 控制吸轮
    public void xiMotor() {
        xiMotorPower = -gamepad2.left_stick_y + xiMotorMinPower;
        robot.xiMotor.setPower(xiMotorPower);
    }

    //彩灯程序
    public void ledControl() {
        if (colorLeft.equals("green") || colorRight.equals("green")) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (colorLeft.equals("purple") || colorRight.equals("purple")) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    // 操控手2的按键控制程序
    public void buttonControlRotateMotor() {
        if (gamepad2.y) {
            while (gamepad2.y) {
                sleep(10);
            }
            flyWheelTargetVelocity += 50;
        }
        if (gamepad2.a) {
            while (gamepad2.a) {
                sleep(10);
            }
            flyWheelTargetVelocity -= 50;
        }
        if (gamepad2.x) {
            while (gamepad2.x) {
                sleep(10);
            }
            flyWheelTargetVelocity = 0;
        }
        if (gamepad2.b) {
            while (gamepad2.b) {
                sleep(10);
            }
            flyWheelTargetVelocity = 1700;
        }


        //控制进球转到下一步
        if (gamepad2.left_bumper) {
            rotateMotorOldTargetPosition = rotateMotorTargetPosition;
            while (gamepad2.left_bumper) sleep(10);
            strikerServoPosition = 0.49;  //一键下降
            sleep(100);
            if (a == 1) rotateMotorTargetPosition += step;//(288/3);
            if (a == 2) rotateMotorTargetPosition += (step + errorPosition);//(288/3);
            a = 1;
            b = 1;
            g = 1;
            p = 1;
        }
        //控制进球装置进入自动旋转模式，直到装满球
        if (gamepad2.left_trigger != 0) {
            rotateMotorOldTargetPosition = rotateMotorTargetPosition;
            while (gamepad2.left_trigger != 0) sleep(10);
            strikerServoPosition = 0.49;  //一键下降
            sleep(100);
            if (a == 1) rotateMotorTargetPosition += step;//(288/3);
            if (a == 2) rotateMotorTargetPosition += (step + errorPosition);//(288/3);
            a = 1;
            b = 2;
            g = 1;
            p = 1;
        }

        //控制射球转到下一步
        if (gamepad2.right_bumper) {
            rotateMotorOldTargetPosition = rotateMotorTargetPosition;
            while (gamepad2.right_bumper) sleep(10);
            strikerServoPosition = strikerServoDownPosition;  //一键下降
            sleep(100);
            if (a == 2) rotateMotorTargetPosition += step;//(288/3);
            if (a == 1) rotateMotorTargetPosition += (step - errorPosition);//(288/3);
            a = 2;
            b = 1;
            g = 1;
            p = 1;
        }
        // 控制射球射球进入自动旋转，直到看到绿色球
        if (gamepad2.dpad_left) {
            rotateMotorOldTargetPosition = rotateMotorTargetPosition;
            while (gamepad2.dpad_left) sleep(10);
            strikerServoPosition = strikerServoDownPosition;  //一键下降
            sleep(100);
            if (a == 2 && (colorLeft.equals("green") || colorRight.equals("green"))) {
                rotateMotorTargetPosition += 0;//(288/3);
            } else {
                if (a == 2) rotateMotorTargetPosition += step;//(288/3);
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);//(288/3);
            }
            a = 2;
            b = 1;
            g = 2;
            p = 1;
        }
        // 控制射球射球进入自动旋转，直到看到紫色球
        if (gamepad2.dpad_right) {
            rotateMotorOldTargetPosition = rotateMotorTargetPosition;
            while (gamepad2.dpad_right) sleep(10);
            strikerServoPosition = strikerServoDownPosition;  //一键下降
            sleep(100);
            if (a == 2 && (colorLeft.equals("purple") || colorRight.equals("purple"))) {
                rotateMotorTargetPosition += 0;//(288/3);
            } else {
                if (a == 2) rotateMotorTargetPosition += step;//(288/3);
                if (a == 1) rotateMotorTargetPosition += (step - errorPosition);//(288/3);
            }
            a = 2;
            b = 1;
            g = 1;
            p = 2;
        }

        //一键转盘初始化
        if (gamepad2.start) {
            runtime.reset();
            while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 3) {
                robot.rotateMotor.setPower(rotatePowerStart);
            }
            robot.rotateMotor.setPower(0);
            sleep(200);
            robot.rotateMotorEncoderRest();
            rotateMotorTargetPosition = 0;
            rotateMotorOldTargetPosition = 0;
            a = 1;
            c = 0;
            b = 1;
            g = 1;
            p = 1;
            sleep(500);  // 等待复位完成
            if (robot.rotateMotor.getCurrentPosition() != 0) {
                telemetry.addData("编码器复位失败", "请检查电机");
                telemetry.update();
                sleep(2000);
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
                while (opModeIsActive()) {

                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    buttonControlRotateMotor();

                    rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();

                    error = rotateMotorTargetPosition - rotateMotorCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) < rotateMotorMaxErrorPosition * 2) ki = 0;
                    ki = Math.max(-ki_max, Math.min(ki, ki_max));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    rotateMotorPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + rotateMotorTargetPosition * kf;

                    double targetPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + rotateMotorTargetPosition * kf;

                    if (Math.abs(error) < 48) { // 接近目标时（阈值根据实际调整）
                        rotateMotorPower = Math.max(-0.3, Math.min(targetPower, 0.3)); // 降低最大功率
                    } else {
                        rotateMotorPower = Math.max(-rotateMotorMaxPower, Math.min(targetPower, rotateMotorMaxPower));
                    }
                    if (Math.abs(rotateMotorPower) < rotateMotorMinPower && Math.abs(error) > rotateMotorMaxErrorPosition) {
                        rotateMotorPower = Math.signum(rotateMotorPower) * rotateMotorMinPower;
                    }

                    if (b == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(300);
                        if ((colorFront.equals("green") || colorFront.equals("purple")) && c < 3) {
                            rotateMotorTargetPosition += step;//(288/3);
                            c += 1;
                            if (c == 3) {
                                b = 1;
                                c = 0;
                            }
                        }
                    } else if (g == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(150);
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            g = 1;
                        } else rotateMotorTargetPosition += step;//(288/3);
                    } else if (p == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(150);
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            p = 1;
                        } else rotateMotorTargetPosition += step;//(288/3);
                    } else {
                        if (Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                            rotateMotorPower = 0;
                        }
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

    public class camera extends Thread {
        public static final boolean USE_WEBCAM = true;
        public AprilTagProcessor aprilTag;
        /**
         * The variable to store our instance of the vision portal.
         */
        public VisionPortal visionPortal;
        public boolean streamingStopped = true;

        public void run() {

            initAprilTag();

            // 不在子线程调用 waitForStart()，避免与主线程冲突

            while (opModeIsActive()) {

                telemetryAprilTag();
                telemetry.update();

                // 安全地控制流（避免重复调用或者在 visionPortal 为 null 时崩溃）
                if (visionPortal != null) {
                    try {
                        if (gamepad1.x) {
                            visionPortal.setProcessorEnabled(aprilTag, false); // 关闭处理器
                            visionPortal.stopStreaming(); // 停止流
                        } else if (gamepad1.y) {
                            visionPortal.setProcessorEnabled(aprilTag, true); // 打开处理器
                            visionPortal.resumeStreaming(); // 如需同时恢复流


                        }
                    } catch (Exception e) {
                        telemetry.addData("Vision error", e.getMessage());
                        telemetry.update();
                    }
                }

                try {
                    sleep(20);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }


        }


        // end method initAprilTag()
        public void initAprilTag() {

            // Create the AprilTag processor.
            // 创建 AprilTag 处理器实例。
            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    // 以下为可选默认设置（按需取消注释并修改）。
                    .setDrawAxes(true) // 是否绘制坐标轴
                    .setDrawCubeProjection(true) // 是否绘制立方体投影
                    .setDrawTagOutline(true) // 是否绘制标签轮廓

                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // 标签族选择
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary()) // 使用的标签库
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES) // 输出单位设置

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    // 如果不手动指定相机内参，SDK 将尝试加载预定义的相机标定参数。
                    .setLensIntrinsics(512.266019477, 512.266019477, 376.90087485, 310.696828493)
                    //fx="512.266019477" fy="512.266019477" cx="376.90087485" cy="310.696828493"
                    // ... these parameters are fx, fy, cx, cy.
                    // ... 这些参数分别为 fx, fy, cx, cy。

                    .build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // 调整图像降采样以在检测距离与检测率之间做权衡。
            // eg: Some typical detection data using a Logitech C920 WebCam
            // 例如：使用 Logitech C920 的典型检测数据
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            // 注意：可在运行时动态调整降采样以适配比赛需求。
            //aprilTag.setDecimation(3);

            // Create the vision portal by using a builder.
            // 使用构建器创建 VisionPortal（视觉入口）。
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            // 设置相机（外接 webcam 或 内置 RC 手机相机）。
            if (USE_WEBCAM) {
                builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            } else {
                builder.setCamera(BuiltinCameraDirection.BACK);
            }

            // 选择相机分辨率。并非所有相机都支持所有分辨率。
            //builder.setCameraResolution(new Size(640, 480));

            // 启用 RC 预览（LiveView）。将其设为 false 可省略相机监视。
            // Enable RC preview (LiveView). Set false to skip camera monitor.
            builder.enableLiveView(true);

            // 设置流格式；MJPEG 相比默认的 YUY2 使用更少带宽。
            // Set stream format; MJPEG uses less bandwidth than default YUY2 in some cases.
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

            // 选择当没有处理器启用时 LiveView 是否停止。
            // 如果设为 true，当没有处理器启用时监视器将显示纯橙色屏幕。
            // 如果设为 false，监视器在没有处理器启用时显示相机视图。
            // Choose whether LiveView stops when no processor is enabled.
            builder.setAutoStopLiveView(true);

            // Set and enable the processor.
            // 添加并启用 AprilTag 处理器。
            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            // 使用上述设置构建 VisionPortal 实例。
            visionPortal = builder.build();

            // Disable or re-enable the aprilTag processor at any time.
            // 可随时禁用或重新启用 aprilTag 处理器。


        }

        public void telemetryAprilTag() {

            // Get detections and report count.
            // 获取当前检测列表并显示数量。
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            // 遍历检测列表并为每个检测项显示信息。
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // If metadata available, show ID, name and pose in FTC coordinate units.
                    // 如果存在元数据，显示 ID、元数据名称和 FTC 坐标系下位姿信息。
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (英寸)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (度)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (英寸, 度, 度)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    // If no metadata, report ID as unknown and show detection center in pixels.
                    // 如果没有元数据，则将 ID 标记为“未知”，并显示检测框中心像素坐标。
                    telemetry.addLine(String.format("\n==== (ID %d) 未知", detection.id));
                    telemetry.addLine(String.format("中心 %6.0f %6.0f   (像素)", detection.center.x, detection.center.y));
                }


            }


            // Add "key" information to telemetry
            // 在 telemetry 中添加说明键，解释各字段含义。
            telemetry.addLine("- XYZ: 3D空间坐标 (X, Y, Z)，单位英寸，表示AprilTag相对于摄像头的位置\n" + "   - PRY: 姿态角 (Pitch, Roll, Yaw)，单位度，表示AprilTag的旋转状态\n" + "   - RBE: 距离-方位-仰角 (Range, Bearing, Elevation)，单位英寸和度，表示AprilTag相对于摄像头的球坐标系位置\n");
        }


    }

}













