package org.firstinspires.ftc.teamcode.FTC_27650_AUTO_Test;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "red_1")
@Config

public class RED_1 extends LinearOpMode {
    public static double greenMin = 140, greenMax = 170;
    MyRobotHardware_27650_Auto robot = new MyRobotHardware_27650_Auto(this);
    camera cameraThread = new camera();
    colorSensor colorSensorThread = new colorSensor();
    servo_xiMotor servo_xiMotorThread = new servo_xiMotor();
    setFlyMotorVelocity setFlyMotorVelocity = new setFlyMotorVelocity();
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    public static double purpleMin = 215, purpleMax = 260;
    public static volatile double flyWheelTargetVelocity = 0;
    public static double fly_kp = 0.01;
    public static double fly_ki = 0.05;
    public static double fly_kd = 0.000005;
    float gain = 3;//颜色传感器增益值，要>=1
    volatile String colorFront = "无";
    volatile String colorLeft = "无";
    volatile String colorRight = "无";
    volatile double distanceFront = 0;
    volatile double distanceLeft = 0;
    volatile double distanceRight = 0;

    volatile double strikerServoDownPosition = 0.43;//角度舵机
    volatile double strikerServoUpPosition = 0.09;//角度舵机
    volatile double strikerServoPosition = strikerServoDownPosition;
    volatile double angleServoPosition = 0; //初始化位置

    volatile boolean camUsing = false;
    volatile double range = 0, angleZ = 0, angleY = 0;
    volatile int id = 0;
    volatile int oldId = 0;
    public static int rotateMotorMaxErrorPosition = 100;
    volatile double flyWheelCurrentVelocity = 0;
    public static int errorPosition = 1422;
    public static double rotate_kp = 0.0006;//0.01;
    public static double rotate_ki = 0.00005;//0.001,
    volatile double xiMotorPower = 0;

    volatile int rotateMotorCurrentPosition = 0;
    volatile int rotateMotorTargetPosition = 0;
    int rotateError = 0;
    volatile double rotateMotorPower = 0;
    public static double rotate_kd = 0.0000006;//0.001;
    public static double ki_max = 5000;
    public static double kf = 0;
    final float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    final float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    final float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    private ElapsedTime runtime = new ElapsedTime();
    volatile int step = 2731;
    double rotateMotorMinPower = 0.1;
    double rotateMotorMaxPower = 0.8;

    volatile int c = 0;
    volatile int b = 1;
    volatile int g = 1;
    volatile int p = 1;

    volatile int upTime = 400;
    volatile int downTime = 400;
    double while_time = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        robot.strikerServo.setPosition(strikerServoPosition);
        robot.angleServo.setPosition(angleServoPosition);
        sleep(200);

        runtime.reset();
        while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 3) {
            robot.rotateMotor.setPower(0.8);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);
        while (robot.magnetic_in.isPressed()) {
            robot.rotateMotor.setPower(0.2);
        }
        robot.rotateMotor.setPower(0);

        robot.rotateMotorEncoderRest();
        sleep(300);

        // 以特定姿势实例化您的 MecanumDrive
        Pose2d initialPose = new Pose2d(-62.20, 37.8, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // actionBuilder 从传递给它的驱动器步骤构建
        TrajectoryActionBuilder seeTag = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-12, 10), Math.toRadians(190));

        TrajectoryActionBuilder she_1 = drive.actionBuilder(new Pose2d(-12, 10, Math.toRadians(190)))
                .turn(Math.toRadians(-57));

        TrajectoryActionBuilder xi_1 = drive.actionBuilder(new Pose2d(-12, 10, Math.toRadians(133)))
                .turn(Math.toRadians(-44))
                .splineTo(new Vector2d(-12, 25), Math.toRadians(89),
                        new TranslationalVelConstraint(25.0))
                .splineTo(new Vector2d(-12, 36), Math.toRadians(89),
                        new TranslationalVelConstraint(10.0))
                .waitSeconds(0.6)
                .splineTo(new Vector2d(-12, 40), Math.toRadians(89),
                        new TranslationalVelConstraint(7.0));
        //.waitSeconds(0.5)
        //.splineTo(new Vector2d(-12, -48), Math.toRadians(271),
        //new TranslationalVelConstraint(7.0));

        TrajectoryActionBuilder she_2 = drive.actionBuilder(new Pose2d(-12, 40, Math.toRadians(89)))
                .strafeToLinearHeading(new Vector2d(-12, 10), Math.toRadians(133));

        telemetry.addData("初始化", "完毕");
        telemetry.update();

        waitForStart();

        // 我们现在处于运行时！我们总是添加以下内容，以便在必要时能够停止机器人。
        if (isStopRequested()) return;

        cameraThread.start();
        colorSensorThread.start();
        servo_xiMotorThread.start();
        setRotateMotorPositionThread.start();
        setFlyMotorVelocity.start();

        camUsing = true;
        runBlocking(new SequentialAction(seeTag.build()));
        sleep(500);
        if (id == 21) oldId = 21;
        if (id == 22) oldId = 22;
        if (id == 23) oldId = 23;
        camUsing = false;
        runBlocking(new SequentialAction(she_1.build()));
        faShe();
        xiBall();
        runBlocking(new SequentialAction(xi_1.build()));
        xiMotorPower = 0;
        runBlocking(new SequentialAction(she_2.build()));
        faShe();
    }
    //相机线程
    public class camera extends Thread {
        public static final boolean USE_WEBCAM = true;
        public AprilTagProcessor aprilTag;
        public VisionPortal visionPortal;
        public boolean streamingStopped = true;
        public void run() {
            initAprilTag();
            // 不在子线程调用 waitForStart()，避免与主线程冲突
            try {
                while (opModeIsActive()) {
                    // 安全地控制流（避免重复调用或者在 visionPortal 为 null 时崩溃）
                    if (visionPortal != null) {
                        try {
                            if (!camUsing) {
                                visionPortal.setProcessorEnabled(aprilTag, false); // 关闭处理器
                                visionPortal.stopStreaming(); // 停止流
                            } else {
                                visionPortal.setProcessorEnabled(aprilTag, true); // 打开处理器
                                visionPortal.resumeStreaming(); // 如需同时恢复流
                                telemetryAprilTag();
                                telemetry.update();
                            }
                        } catch (Exception e) {
                            telemetry.addData("Vision error", e.getMessage());
                            telemetry.update();
                        }
                    }
                    sleep(20);
                }
                // 当不再需要相机时，节省更多CPU资源。
                visionPortal.close();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        public void initAprilTag() {

            // 创建 AprilTag 处理器实例。
            aprilTag = new AprilTagProcessor.Builder()
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
            builder.enableLiveView(true);

            // 设置流格式；MJPEG 相比默认的 YUY2 使用更少带宽。
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

            // 选择当没有处理器启用时 LiveView 是否停止。
            // 如果设为 true，当没有处理器启用时监视器将显示纯橙色屏幕。
            // 如果设为 false，监视器在没有处理器启用时显示相机视图。
            // Choose whether LiveView stops when no processor is enabled.
            builder.setAutoStopLiveView(true);

            // 添加并启用 AprilTag 处理器。
            builder.addProcessor(aprilTag);

            // 使用上述设置构建 VisionPortal 实例。
            visionPortal = builder.build();

            // Disable or re-enable the aprilTag processor at any time.
            // 可随时禁用或重新启用 aprilTag 处理器。


        }
        public void telemetryAprilTag() {
            // 获取当前检测列表并显示数量。
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            // 遍历检测列表并为每个检测项显示信息。
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // 如果存在元数据，显示 ID、元数据名称和 FTC 坐标系下位姿信息。
                    id = detection.id;
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (英寸)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (度)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    //射程、方位角和仰角
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (英寸, 度, 度)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    range = detection.ftcPose.range;
                    angleZ = detection.ftcPose.bearing;
                    angleY = detection.ftcPose.elevation;
                    telemetry.addData("射球角度", "%4.2f", angleServoPosition);
                    telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), robot.flyWheelLeft.getVelocity());
                    telemetry.addData("右飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelRight.getPower(), robot.flyWheelRight.getVelocity());
                    telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
                    telemetry.addData("oldID", "%7d", oldId);
                } else {
                    // 如果没有元数据，则将 ID 标记为“未知”，并显示检测框中心像素坐标。
                    telemetry.addLine(String.format("\n==== (ID %d) 未知", detection.id));
                    telemetry.addLine(String.format("中心 %6.0f %6.0f   (像素)", detection.center.x, detection.center.y));
                }
            }
            // Add "key" information to telemetry
            // 在 telemetry 中添加说明键，解释各字段含义。
            //telemetry.addLine("- XYZ: 3D空间坐标 (X, Y, Z)，单位英寸，表示AprilTag相对于摄像头的位置\n" + "   - PRY: 姿态角 (Pitch, Roll, Yaw)，单位度，表示AprilTag的旋转状态\n" + "   - RBE: 距离-方位-仰角 (Range, Bearing, Elevation)，单位英寸和度，表示AprilTag相对于摄像头的球坐标系位置\n");
        }
    }
    public class colorSensor extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
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
                    if ((hsvValuesFront[2] != 0 || hsvValuesFront[1] != 0 || hsvValuesFront[0] != 0)
                            && distanceFront <= 2.5) {
                        if ((greenMin <= hsvValuesFront[0] && hsvValuesFront[0] <= greenMax)
                                && (0.15 < hsvValuesFront[1] && hsvValuesFront[1] < 0.95)
                                && hsvValuesFront[1] > 0.01) { // 增加饱和度阈值
                            colorFront = "green";
                        } else if ((purpleMin <= hsvValuesFront[0] && hsvValuesFront[0] <= purpleMax)
                                && (0.15 < hsvValuesFront[1] && hsvValuesFront[1] < 0.95)
                                && hsvValuesFront[1] > 0.01) {
                            colorFront = "purple";
                        } else {
                            colorFront = "有";
                        }
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

                    if (!camUsing) show();
                    //sleep(5);
                }
                sleep(5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public class servo_xiMotor extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {
                    robot.strikerServo.setPosition(strikerServoPosition);
                    robot.angleServo.setPosition(angleServoPosition);
                    robot.xiMotor.setPower(xiMotorPower);
                    //sleep(5);
                }
                sleep(5);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public class setFlyMotorVelocity extends Thread {
        public void run() {
            try {
                double error = 0;
                double lastError = 0;
                double ki = 0;
                double kd = 0;
                // 在setRotateMotorPositionThread中使用固定周期控制
                ElapsedTime loopTimer = new ElapsedTime();
                final double loopPeriod = 0.01; // 10ms周期
                while (opModeIsActive()) {

                    double dt = loopTimer.seconds();
                    loopTimer.reset();

                    flyWheelCurrentVelocity = 0.5 * (robot.flyWheelLeft.getVelocity() + robot.flyWheelRight.getVelocity());

                    error = flyWheelTargetVelocity - flyWheelCurrentVelocity;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) < rotateMotorMaxErrorPosition * 2) ki = 0;
                    //ki = Math.max(-ki_max, Math.min(ki, ki_max));  // 根据实际情况调整上下限
                    kd = (error - lastError) / dt;

                    double flyPower = error * fly_kp + ki * fly_ki + kd * fly_kd;

                    //robot.flyWheelLeft.setPower(flyPower);
                    //robot.flyWheelRight.setPower(flyPower);
                    robot.flyWheelLeft.setVelocity(flyWheelTargetVelocity);
                    robot.flyWheelRight.setVelocity(flyWheelTargetVelocity);
                    lastError = error;

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

                    rotateMotorCurrentPosition = -robot.rotateMotor.getCurrentPosition();

                    error = rotateMotorTargetPosition - rotateMotorCurrentPosition;
                    ki += error * dt;
                    // 误差过小时清除积分（避免静态误差累积）
                    if (Math.abs(error) < rotateMotorMaxErrorPosition * 2) ki = 0;
                    ki = Math.max(-ki_max, Math.min(ki, ki_max));  // 根据实际情况调整上下限
                    kd = (error - old_error) / dt;
                    rotateMotorPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + rotateMotorTargetPosition * kf;
                    rotateMotorPower = Math.max(-rotateMotorMaxPower, Math.min(rotateMotorPower, rotateMotorMaxPower));
                    /*
                    double targetPower = error * rotate_kp + ki * rotate_ki + kd * rotate_kd + rotateMotorTargetPosition * kf;
                    if (Math.abs(error) < 48) { // 接近目标时（阈值根据实际调整）
                        rotateMotorPower = Math.max(-0.3, Math.min(targetPower, 0.3)); // 降低最大功率
                    } else {
                        rotateMotorPower = Math.max(-rotateMotorMaxPower, Math.min(targetPower, rotateMotorMaxPower));
                    }
                    if (Math.abs(rotateMotorPower) < rotateMotorMinPower && Math.abs(error) > rotateMotorMaxErrorPosition) {
                        rotateMotorPower = Math.signum(rotateMotorPower) * rotateMotorMinPower;
                    }

                    */

                    if (b == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(300);
                        if ((colorFront.equals("green") || colorFront.equals("purple") || colorFront.equals("有"))) {
                            rotateMotorTargetPosition += step;//(288/3);
                            c += 1;
                            if (c == 2) {
                                b = 1;
                                c = 0;
                            }
                        }
                    } else if (g == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(300);
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            g = 1;
                        } else if (colorLeft.equals("有") && colorRight.equals("有")) {
                            strikerServoPosition = 0.2;//0.56下
                            sleep(200);
                            strikerServoPosition = strikerServoDownPosition;//0.56下
                            sleep(200);
                        } else {
                            rotateMotorTargetPosition += step;//(288/3);
                        }
                    } else if (p == 2 && Math.abs(error) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(300);
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            p = 1;
                        } else if (colorLeft.equals("有") && colorRight.equals("有")) {
                            strikerServoPosition = 0.2;//0.56下
                            sleep(200);
                            strikerServoPosition = strikerServoDownPosition;//0.56下
                            sleep(200);
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

    public void xiBall() {
        rotateMotorTargetPosition += (step + errorPosition);//转到准备发射位置
        sleep(1000);
        xiMotorPower = 1.0;
        b = 2;
        c = 0;
    }

    public void show() {
        telemetry.clear();
        telemetry.addData("ID", "%7d", oldId);
        telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
        telemetry.addData("左飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelLeft.getPower(), robot.flyWheelLeft.getVelocity());
        telemetry.addData("右飞轮功率/转速", "%4.2f, %4.2f", robot.flyWheelRight.getPower(), robot.flyWheelRight.getVelocity());
        telemetry.addData("旋转误差", "%7d", rotateError);
        telemetry.addData("旋转位置", "%7d", rotateMotorCurrentPosition);
        telemetry.addData("目标位置 ", "%7d", rotateMotorTargetPosition);
        telemetry.addData("g / p", "%7d, %7d", g, p);
        telemetry.update();
    }

    public void faShe() {
        flyWheelTargetVelocity = 1550;
        angleServoPosition = 0.6;
        rotateMotorTargetPosition += (step - errorPosition);//转到准备发射位置
        sleep(800);
        if (oldId == 21) {
            greenBall();
            purpleBall();
            purpleBall();
        }
        if (oldId == 22) {
            purpleBall();
            greenBall();
            purpleBall();
        }
        if (oldId == 23) {
            purpleBall();
            purpleBall();
            greenBall();
        }
        flyWheelTargetVelocity = 0;
    }

    public void greenBall() {
        g = 2;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= while_time) {
            if (g == 1) break;
        }
        g = 1;
        strikerServoPosition = strikerServoUpPosition;
        sleep(upTime);
        strikerServoPosition = strikerServoDownPosition;
        sleep(downTime);
    }

    public void purpleBall() {
        p = 2;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() <= while_time) {
            if (p == 1) break;
        }
        p = 1;
        strikerServoPosition = strikerServoUpPosition;
        sleep(upTime);
        strikerServoPosition = strikerServoDownPosition;
        sleep(downTime);
    }
}
