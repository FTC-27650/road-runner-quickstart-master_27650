package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "手动27650")
@Config
public class MecanumWheel extends LinearOpMode {
    public static double strikerServoSpeed = 0.0005;
    public static double greenMin = 140, greenMax = 190;
    public static double purpleMin = 225, purpleMax = 250;
    public static double rotatePowerStart = 0.15;
    public static double rotateMotorMinPower = 0.3;
    public static double flyWheelMinPower = 0;
    public static double xiMotorMinPower = 0;
    final float[] hsvValuesFront = new float[3]; // 1前面色调 2饱和度
    final float[] hsvValuesLeft = new float[3]; // 1左边色调 2饱和度
    final float[] hsvValuesRight = new float[3];// 1右边色调 2饱和度
    public String colorFront = "无";
    public String colorLeft = "无";
    public String colorRight = "无";
    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);
    Mecanum_1Thread Mecanum_1Thread = new Mecanum_1Thread();
    greenBallThread greenBallThread = new greenBallThread();
    purpleBallThread purpleBallThread = new purpleBallThread();
    inBallPrepareThread inBallPrepareThread = new inBallPrepareThread();
    flyWheelThread flyWheelThread = new flyWheelThread();
    xiMotorThread xiMotorThread = new xiMotorThread();
    boolean greenBallThreadUsing = true;
    boolean purpleBallThreadUsing = true;
    boolean inBallPrepareThreadUsing = true;
    double strikerServoPosition = 0.49;//角度舵机    0.49代表转到中间
    boolean servoUsing = true;
    float gain = 2;//颜色传感器增益值，要>=1
    boolean magnetic_out_bool = false;
    boolean magnetic_in_bool = false;
    int rotateMotorCurrentPosition = 0;
    int rotateMotorTargetPosition = 0;
    double rotateMotorPower = 0;
    double flyWheelPower = 0;
    double xiMotorPower = 0;
    int a = 1;
    int step = 96;

    @Override
    public void runOpMode() {

        robot.init();
        robot.strikerServo.setPosition(strikerServoPosition);
        sleep(200);

        while (!robot.magnetic_in.isPressed()) {
            robot.rotateMotor.setPower(rotatePowerStart);
        }
        robot.rotateMotor.setPower(0);

        robot.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        Mecanum_1Thread.start();
        flyWheelThread.start();
        xiMotorThread.start();
        while (opModeIsActive()) {

            if (gamepad2.left_bumper) {
                purpleBallThreadUsing = false;
                inBallPrepareThreadUsing = false;
                while (gamepad2.left_bumper) {
                    if (!gamepad2.left_bumper) {
                        break;
                    }
                }
                greenBallThread.start();
                purpleBallThreadUsing = true;
                inBallPrepareThreadUsing = true;
            }
            if (gamepad2.right_bumper) {
                greenBallThreadUsing = false;
                inBallPrepareThreadUsing = false;
                while (gamepad2.right_bumper) {
                    if (!gamepad2.right_bumper) {
                        break;
                    }
                }
                purpleBallThread.start();
                greenBallThreadUsing = true;
                inBallPrepareThreadUsing = true;
            }
            if (gamepad2.b) {
                greenBallThreadUsing = false;
                purpleBallThreadUsing = false;
                while (gamepad2.b) {
                    if (!gamepad2.b) {
                        break;
                    }
                }
                inBallPrepareThread.start();
                greenBallThreadUsing = true;
                purpleBallThreadUsing = true;
            }
            if (servoUsing) servoControl();
            colorSensor();
            show();
        }
    }

    public void show() {
        //magnetic_out_bool = robot.magnetic_out.isPressed();
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
        telemetry.addData("目标位置", "%7d", rotateMotorTargetPosition);
        telemetry.addData("飞轮功率", "%4.2f", flyWheelPower);
        telemetry.addData("旋吸功率", "%4.2f", xiMotorPower);
        telemetry.update();
    }

    public void servoControl() {
        if (gamepad2.dpad_up)
            strikerServoPosition = Math.min(strikerServoPosition + strikerServoSpeed, 1);
        if (gamepad2.dpad_down)
            strikerServoPosition = Math.max(strikerServoPosition - strikerServoSpeed, 0);

        if (gamepad2.y) strikerServoPosition = 0.72;  //一键抬升
        if (gamepad2.a) strikerServoPosition = 0.49;  //一键下降
//robot.rotateServo.setPosition(rotateServoPosition);
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

    public class Mecanum_1Thread extends Thread {
        public void run() {
            while (opModeIsActive()) {
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
                    frPower = flPower * 0.7;
                    brPower = flPower * 0.7;
                    blPower = flPower * 0.7;

                }

                robot.fl.setPower(flPower);
                robot.fr.setPower(frPower);
                robot.br.setPower(brPower);
                robot.bl.setPower(blPower);
            }
        }
    }

    public class flyWheelThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                flyWheelPower = -gamepad2.left_stick_y + flyWheelMinPower;
                robot.flyWheelLeft.setPower(flyWheelPower);
                robot.flyWheelRight.setPower(flyWheelPower);
            }
        }
    }

    public class xiMotorThread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                xiMotorPower = -gamepad2.right_stick_y + xiMotorMinPower;
                robot.xiMotor.setPower(xiMotorPower);
            }
        }
    }

    public class greenBallThread extends Thread {
        public void run() {
            try {
                servoUsing = false;
                strikerServoPosition = 0.49;  //一键下降
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(200);
                if (a == 2 && (colorLeft.equals("green") || colorRight.equals("green"))) {
                    rotateMotorTargetPosition += 0;//(288/3);
                } else {
                    if (a == 2) {
                        rotateMotorTargetPosition += step;//(288/3);
                    }
                    if (a == 1) {
                        rotateMotorTargetPosition += (step - 20);//(288/3);
                    }
                    while (opModeIsActive() && greenBallThreadUsing) {
                        rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
                        rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * 0.01;
                        if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower) {
                            rotateMotorPower = rotateMotorMinPower;
                        }
                        if (rotateMotorPower < 0 && rotateMotorPower >= -rotateMotorMinPower) {
                            rotateMotorPower = -rotateMotorMinPower;
                        }

                        robot.rotateMotor.setPower(rotateMotorPower);

                        if (Math.abs(rotateMotorTargetPosition - rotateMotorCurrentPosition) <= 5 || rotateMotorCurrentPosition > rotateMotorTargetPosition - 5) {
                            rotateMotorPower = 0;
                            robot.rotateMotor.setPower(rotateMotorPower);
                            sleep(100);
                            if (colorLeft.equals("green") || colorRight.equals("green")) {
                                break;
                            } else {
                                rotateMotorTargetPosition += step;//(288/3);
                            }
                        }

                    }
                    rotateMotorPower = 0;
                    robot.rotateMotor.setPower(rotateMotorPower);
                }
                a = 2;
                servoUsing = true;
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public class purpleBallThread extends Thread {
        public void run() {
            try {
                servoUsing = false;
                strikerServoPosition = 0.49;  //一键下降
                robot.strikerServo.setPosition(strikerServoPosition);
                sleep(200);
                if (a == 2 && (colorLeft.equals("purple") || colorRight.equals("purple"))) {
                    rotateMotorTargetPosition += 0;//(288/3);
                } else {
                    if (a == 2) {
                        rotateMotorTargetPosition += step;//(288/3);
                    }
                    if (a == 1) {
                        rotateMotorTargetPosition += (step - 20);//(288/3);
                    }
                    while (opModeIsActive() && purpleBallThreadUsing) {
                        rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
                        rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * 0.01;
                        if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower) {
                            rotateMotorPower = rotateMotorMinPower;
                        }
                        if (rotateMotorPower < 0 && rotateMotorPower >= -rotateMotorMinPower) {
                            rotateMotorPower = -rotateMotorMinPower;
                        }

                        robot.rotateMotor.setPower(rotateMotorPower);

                        if (Math.abs(rotateMotorTargetPosition - rotateMotorCurrentPosition) <= 5 || rotateMotorCurrentPosition > rotateMotorTargetPosition - 5) {
                            rotateMotorPower = 0;
                            robot.rotateMotor.setPower(rotateMotorPower);
                            sleep(100);
                            if (colorLeft.equals("purple") || colorRight.equals("purple")) {
                                break;
                            } else {
                                rotateMotorTargetPosition += step;//(288/3);
                            }
                        }
                    }
                }
                rotateMotorPower = 0;
                robot.rotateMotor.setPower(rotateMotorPower);
                a = 2;
                servoUsing = true;
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public class inBallPrepareThread extends Thread {

        public void run() {
            servoUsing = false;
            strikerServoPosition = 0.49;  //一键下降
            robot.strikerServo.setPosition(strikerServoPosition);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            if (a == 1) {
                rotateMotorTargetPosition += step;//(288/3);
            }
            if (a == 2) {
                rotateMotorTargetPosition += (step + 20);//(288/3);
            }

            while (opModeIsActive() && inBallPrepareThreadUsing) {
                rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
                rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * 0.01;
                if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower) {
                    rotateMotorPower = rotateMotorMinPower;
                }
                if (rotateMotorPower < 0 && rotateMotorPower >= -rotateMotorMinPower) {
                    rotateMotorPower = -rotateMotorMinPower;
                }
                if (Math.abs(rotateMotorTargetPosition - rotateMotorCurrentPosition) <= 5 || rotateMotorCurrentPosition > rotateMotorTargetPosition - 5) {
                    break;
                }
                robot.rotateMotor.setPower(rotateMotorPower);
            }
            rotateMotorPower = 0;
            robot.rotateMotor.setPower(rotateMotorPower);
            a = 1;
            servoUsing = true;
        }
    }
}
