package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "手动27650——new_4", group = "Linear Opmode")
@Config
public class MecanumWheel_new_4 extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    public static double strikerServoDownPosition = 0.55;//角度舵机    0.49代表转到中间
    public static double strikerServoUpPosition = 0.8;//角度舵机    0.49代表转到中间
    public static double greenMin = 140, greenMax = 195;
    public static double purpleMin = 215, purpleMax = 260;
    public static int rotateMotorOldTargetPosition = 0;
    public static int rotateMotorTargetPosition = 0;
    public static int rotateMotorMaxErrorPosition = 5;
    public static double rotatePowerStart = 0.1;
    public static double rotateMotorPower = 0;
    public static double rotateMotorMinPower = 0.2;
    public static double rotateMotorMaxPower = 0.5;
    public static int errorPosition = 21;
    public static double xiMotorMinPower = 0;
    final float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    final float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    final float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    private final ElapsedTime runtime = new ElapsedTime();
    public String colorFront = "无";
    public String colorLeft = "无";
    public String colorRight = "无";
    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);
    setRotateMotorPositionThread setRotateMotorPositionThread = new setRotateMotorPositionThread();
    flyWheelThread flyWheelThread = new flyWheelThread();
    double strikerServoPosition = strikerServoDownPosition;
    float gain = 3;//颜色传感器增益值，要>=1
    boolean magnetic_out_bool = false;
    boolean magnetic_in_bool = false;
    int rotateMotorCurrentPosition = 0;
    double kp = 0;
    double flyWheelPower = 0;
    double flyWheelCurrentVelocity = 0;
    double xiMotorPower = 0;
    int a = 1;
    int b = 1;
    int c = 0;
    int g = 1;
    int p = 1;
    int step = 96;
    double flyWheelVelocity = 0;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        robot.init();
        initAprilTag();
        robot.strikerServo.setPosition(strikerServoPosition);
        sleep(200);
        runtime.reset();
        while (!robot.magnetic_in.isPressed() && runtime.seconds() <= 3) {
            robot.rotateMotor.setPower(rotatePowerStart);
        }
        robot.rotateMotor.setPower(0);
        sleep(200);
        robot.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //setRotateMotorPositionThread.start();
        flyWheelThread.start();
        while (opModeIsActive()) {
            rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
            servoControl();
            colorSensor();
            Mecanum();

            xiMotor();
            ledControl();
            show();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (英寸)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (度)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (英寸, 度, 度)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) 未知", detection.id));
                telemetry.addLine(String.format("中心 %6.0f %6.0f   (像素)", detection.center.x, detection.center.y));
            }
        }


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public void show() {

        magnetic_in_bool = robot.magnetic_in.isPressed();

        telemetry.addData("imu", "%4.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("向上抬球", "%4.2f", strikerServoPosition);
        telemetry.addData("Gain", gain);
        telemetry.addData("色调 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[0], hsvValuesLeft[0], hsvValuesRight[0]);
        telemetry.addData("饱和度 前/左/右", "%.3f,%.3f, %.3f", hsvValuesFront[1], hsvValuesLeft[1], hsvValuesRight[1]);
        telemetry.addData("球颜色 前/左/右", "%s, %s, %s", colorFront, colorLeft, colorRight);
        telemetry.addData("磁性限位开关 out/in", "%b, %b", magnetic_out_bool, magnetic_in_bool);
        telemetry.addData("旋转功率", "%4.2f", rotateMotorPower);
        telemetry.addData("旋转位置", "%7d", rotateMotorCurrentPosition);
        telemetry.addData("目标位置 old/new/差值", "%7d ,%7d, %7d", rotateMotorOldTargetPosition, rotateMotorTargetPosition, rotateMotorTargetPosition - rotateMotorOldTargetPosition);
        telemetry.addData("飞轮功率/转速", "%4.2f, %4.2f", flyWheelPower, robot.flyWheelLeft.getVelocity());
        telemetry.addData("旋吸功率", "%4.2f", xiMotorPower);
        telemetry.addData("a,b,c,kp", "%d ,%d, %d, %4.2f", a, b, c, kp);
        telemetry.addData("g, p", "%d, %d", g, p);
        telemetry.addData("发射器转速", "%4.2f", flyWheelCurrentVelocity);
        telemetry.addData("gamepad2.fuck", "%4.2f", -gamepad2.right_stick_y);
        telemetryAprilTag();
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
            flPower = flPower * 0.7;
            frPower = frPower * 0.7;
            brPower = brPower * 0.7;
            blPower = blPower * 0.7;

        }

        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.br.setPower(brPower);
        robot.bl.setPower(blPower);
    }
    // end method telemetryAprilTag()

    public void servoControl() {


        if (gamepad2.dpad_up) strikerServoPosition = strikerServoUpPosition;  //一键抬升
        if (gamepad2.dpad_down) strikerServoPosition = strikerServoDownPosition;  //一键下降

        robot.strikerServo.setPosition(strikerServoPosition);
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

        //前边颜色传感器
        if ((greenMin <= hsvValuesFront[0] && hsvValuesFront[0] <= greenMax) && hsvValuesFront[1] != 1)
            colorFront = "green";
        else if ((purpleMin <= hsvValuesFront[0] && hsvValuesFront[0] <= purpleMax) && hsvValuesFront[1] != 1)
            colorFront = "purple";
        else colorFront = "无";

        //左边颜色传感器
        if ((greenMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= greenMax) && hsvValuesLeft[1] != 1)
            colorLeft = "green";
        else if ((purpleMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= purpleMax) && hsvValuesLeft[1] != 1)
            colorLeft = "purple";
        else colorLeft = "无";

        //右边颜色传感器
        if ((greenMin <= hsvValuesRight[0] && hsvValuesRight[0] <= greenMax) && hsvValuesRight[1] != 1)
            colorRight = "green";
        else if ((purpleMin <= hsvValuesRight[0] && hsvValuesRight[0] <= purpleMax) && hsvValuesRight[1] != 1)
            colorRight = "purple";
        else colorRight = "无";
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
            robot.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotateMotorTargetPosition = 0;
            sleep(200);
        }
    }

    public class setRotateMotorPositionThread extends Thread {
        public void run() {
            try {
                while (opModeIsActive()) {

                    buttonControlRotateMotor();

                    if (rotateMotorTargetPosition - rotateMotorOldTargetPosition >= step * 1.5)
                        kp = 0.0075;
                    else kp = 0.01;
                    rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * kp;
                    if (rotateMotorPower > 0)
                        rotateMotorPower = Math.max(rotateMotorMinPower, Math.min(rotateMotorPower, rotateMotorMaxPower));
                    if (rotateMotorPower < 0)
                        rotateMotorPower = Math.min(-rotateMotorMinPower, Math.max(rotateMotorPower, -rotateMotorMaxPower));

                    if (b == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
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
                        sleep(100);
                        if (colorLeft.equals("green") || colorRight.equals("green")) {
                            g = 1;
                        } else rotateMotorTargetPosition += step;//(288/3);
                    } else if (p == 2 && Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                        rotateMotorPower = 0;
                        robot.rotateMotor.setPower(rotateMotorPower);
                        sleep(100);
                        if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                            p = 1;
                        } else rotateMotorTargetPosition += step;//(288/3);
                    } else {
                        if (Math.abs(rotateMotorCurrentPosition - rotateMotorTargetPosition) <= rotateMotorMaxErrorPosition) {
                            rotateMotorPower = 0;
                        }
                    }
                    robot.rotateMotor.setPower(rotateMotorPower);
                    sleep(10);
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    //飞轮线程 gamepad2.right_stick_y 控制飞轮
    public class flyWheelThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                double MAX_FLYWHEEL_VELOCITY = 4000;
                flyWheelCurrentVelocity = robot.flyWheelLeft.getVelocity();
                flyWheelVelocity = -gamepad2.right_stick_y * MAX_FLYWHEEL_VELOCITY;
                robot.flyWheelLeft.setVelocity(flyWheelVelocity);


            }
        }
    }
}

