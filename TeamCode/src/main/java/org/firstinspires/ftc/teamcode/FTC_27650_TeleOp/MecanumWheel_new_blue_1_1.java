package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "手动27650——new_test", group = "LinearOpmode")
@Config
//@Disabled
public class MecanumWheel_new_blue_1_1 extends LinearOpMode {


    public static double flyWheelTargetVelocity = 1420;
    public static double range = 30;
    public static double angleServoPosition = 0.36; //初始化位置
    volatile boolean camUsing = false;
    volatile int id = 0;
    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);
    camera camera = new camera();

    @Override
    public void runOpMode() {

        robot.init();

        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();
        camera.start();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                robot.angleServo.setPosition(angleServoPosition);
                robot.flyWheelLeft.setVelocity(flyWheelTargetVelocity);
                robot.flyWheelRight.setVelocity(flyWheelTargetVelocity);
            }

        }
    }

    public class camera extends Thread {
        public static final boolean USE_WEBCAM = true;
        public AprilTagProcessor aprilTag;
        public VisionPortal visionPortal;

        public void run() {
            initAprilTag();
            show();
            // 不在子线程调用 waitForStart()，避免与主线程冲突
            try {
                while (opModeIsActive()) {

                    if (camUsing) {

                        telemetry.update();
                        if (aprilTag.getDetections().size() == 1 && id == 20) {
                            if (range < 33.5) {
                                flyWheelTargetVelocity = 1420;

                            } else if (33.5 <= range && range <= 95) {
                                flyWheelTargetVelocity = 1220 + (range) * 4.5;


                            } else if (115 <= range) {
                                flyWheelTargetVelocity = 2025;

                            }
                        }

                    }
                    // 安全地控制流（避免重复调用或者在 visionPortal 为 null 时崩溃）
                    if (visionPortal != null) {
                        try {
                            if (gamepad2.x) {
                                camUsing = false;
                                visionPortal.setProcessorEnabled(aprilTag, false); // 关闭处理器
                                visionPortal.stopStreaming(); // 停止流
                            } else if (gamepad2.y) {
                                camUsing = true;
                                visionPortal.setProcessorEnabled(aprilTag, true); // 打开处理器
                                visionPortal.resumeStreaming(); // 如需同时恢复流
                            }
                        } catch (Exception e) {
                            telemetry.addData("Vision error", e.getMessage());
                            telemetry.update();
                        }
                    }
                    sleep(20);
                }
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

        public void show() {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    // 如果存在元数据，显示 ID、元数据名称和 FTC 坐标系下位姿信息。
                    id = detection.id;
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (英寸, 度, 度)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    // 如果没有元数据，则将 ID 标记为“未知”，并显示检测框中心像素坐标。
                    telemetry.addLine(String.format("\n==== (ID %d) 未知", detection.id));
                    telemetry.addLine(String.format("中心 %6.0f %6.0f   (像素)", detection.center.x, detection.center.y));
                }
            }
        }


    }


}













