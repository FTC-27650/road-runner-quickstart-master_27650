package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@TeleOp(name = "手动27650")
@Config

public class MecanumWheel extends LinearOpMode {
    public static double strikerServoSpeed = 0.0001;
    public static double greenMin = 152, greenMax = 185;
    public static double purpleMin = 225, purpleMax = 250;
    public static double rotatePowerStart = 0.2;
    public static double rotateMotorMinPower = 0.6;
    public static double shooterMotorPower = 0;
    final float[] hsvValuesLeft = new float[3]; //左边色调
    final float[] hsvValuesRight = new float[3];//右边色调


    public String colorLeft = "无";
    public String colorRight = "无";
    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);
    Mecanum_1Thread Mecanum_1Thread = new Mecanum_1Thread();
    inBallPrepareThread inBallPrepareThread = new inBallPrepareThread();
    outBallPrepareThread outBallPrepareThread = new outBallPrepareThread();
    shooterThread shooterThread = new shooterThread();

    double strikerServoPosition = 0.6;//角度舵机    0.5代表转到中间
    boolean servoUsing = true;
    float gain = 2;//颜色传感器增益值，要>=1
    boolean magnetic_out_bool = false;
    boolean magnetic_in_bool = false;

    int rotateMotorTargetPosition = 0;
    double rotateMotorPower = 0;
    double collectMotorPower = 0;
    int a = 1;
    int b = 1;
    double rotateMotorCurrentPosition = 0;
    final int step = 96;

    @Override
    public void runOpMode() {

        robot.init();
        rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
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
        //inBallPrepareThread.start();
        while (opModeIsActive()) {
            rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();


            if (gamepad2.left_bumper) {
                while (gamepad2.left_bumper) {
                    if (!gamepad2.left_bumper) {
                        break;
                    }
                }
                inBallPrepareThread.start();


            }
            if (gamepad2.right_bumper) {
                while (gamepad2.right_bumper) {
                    if (!gamepad2.right_bumper) {
                        break;
                    }
                }
                outBallPrepareThread.start();

            }
            if (gamepad2.right_trigger >= 0.2) {
                shooterThread.start();
            }
            if (servoUsing) servoControl();
            colorSensor();
            motorControl();
            show();
        }
    }

    public void show() {
        magnetic_out_bool = robot.magnetic_out.isPressed();
        magnetic_in_bool = robot.magnetic_in.isPressed();

        telemetry.addData("向上抬球", "%4.2f", strikerServoPosition);
        telemetry.addData("Gain", gain);
        telemetry.addData("色调 左/右", "%.3f, %.3f", hsvValuesLeft[0], hsvValuesRight[0]);
        telemetry.addData("球颜色 左/右", "%s, %s", colorLeft, colorRight);
        telemetry.addData("磁性限位开关 out/in", "%b, %b", magnetic_out_bool, magnetic_in_bool);
        telemetry.addData("旋转功率", "%4.2f", rotateMotorPower);
        telemetry.addData("旋转位置", "%7d", rotateMotorCurrentPosition);
        telemetry.addData("目标位置", "%7d", rotateMotorTargetPosition);
        telemetry.update();
    }

    public void servoControl() {
        if (gamepad2.dpad_up)
            strikerServoPosition = Math.min(strikerServoPosition + strikerServoSpeed, 1);
        if (gamepad2.dpad_down)
            strikerServoPosition = Math.max(strikerServoPosition - strikerServoSpeed, 0);

        if (gamepad2.y) strikerServoPosition = 0.3;  //一键抬升
        if (gamepad2.a) strikerServoPosition = 0.6;  //一键下降
        robot.strikerServo.setPosition(strikerServoPosition);
    }

    public void colorSensor() {
        if (gamepad1.a) gain += 0.005F;
        else if (gamepad1.b && gain > 1) gain -= 0.005F;
        robot.colorSensorLeft.setGain(gain);
        robot.colorSensorRight.setGain(gain);

        NormalizedRGBA colorsLeft = robot.colorSensorLeft.getNormalizedColors();
        NormalizedRGBA colorsRight = robot.colorSensorRight.getNormalizedColors();

        Color.colorToHSV(colorsLeft.toColor(), hsvValuesLeft);
        Color.colorToHSV(colorsRight.toColor(), hsvValuesRight);

        if (greenMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= greenMax) colorLeft = "green";
        else if (purpleMin <= hsvValuesLeft[0] && hsvValuesLeft[0] <= purpleMax)
            colorLeft = "purple";
        else colorLeft = "无";
        if (greenMin <= hsvValuesRight[0] && hsvValuesRight[0] <= greenMax) colorRight = "green";
        else if (purpleMin <= hsvValuesRight[0] && hsvValuesRight[0] <= purpleMax)
            colorRight = "purple";
        else colorRight = "无";
    }
    public void motorControl() {
        if(!gamepad2.left_bumper && !gamepad2.right_bumper){
            collectMotorPower = 0;
        }
        if(gamepad2.left_bumper){
            collectMotorPower = 1;
        }
        if(gamepad2.right_bumper){
            collectMotorPower = -1;
        }
        robot.collectMotor.setPower(collectMotorPower);
        robot.shooterMotor1.setPower(shooterMotorPower);
        robot.shooterMotor2.setPower(shooterMotorPower);

    }


    public class Mecanum_1Thread extends Thread {
        public void run() {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                double flPower = y + x + rx;
                double frPower = y - x - rx;
                double brPower = y + x - rx;
                double blPower = y - x + rx;

                robot.fl.setPower(flPower);
                robot.fr.setPower(frPower);
                robot.br.setPower(brPower);
                robot.bl.setPower(blPower);
            }
        }
    }

    public class inBallPrepareThread extends Thread {

        public void run() {

            strikerServoPosition = 0.6;  //一键下降
            robot.strikerServo.setPosition(strikerServoPosition);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            if (a == 1) {
                rotateMotorTargetPosition += step;//(288/3);
            }


            while (opModeIsActive()) {
                rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * 0.01;

                if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower) {
                    rotateMotorPower = rotateMotorMinPower;
                }
                if (rotateMotorPower < 0 && rotateMotorPower >= -rotateMotorMinPower) {
                    rotateMotorPower = -rotateMotorMinPower;
                }
                if (Math.abs(rotateMotorTargetPosition - rotateMotorCurrentPosition) <= 5) {

                    break;
                }

                robot.rotateMotor.setPower(rotateMotorPower);
            }
            rotateMotorPower = 0;

            robot.rotateMotor.setPower(rotateMotorPower);
            a = 1;
        }
    }

    public class outBallPrepareThread extends Thread {
        public void run() {

            strikerServoPosition = 0.6;  //一键下降
            robot.strikerServo.setPosition(strikerServoPosition);
            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            if (b == 1) {
                rotateMotorTargetPosition += step + 20;//(288/3);
            }


            while (opModeIsActive()) {
                rotateMotorPower = (rotateMotorTargetPosition - rotateMotorCurrentPosition) * 0.01;

                if (0 < rotateMotorPower && rotateMotorPower <= rotateMotorMinPower) {
                    rotateMotorPower = rotateMotorMinPower;
                }
                if (rotateMotorPower < 0 && rotateMotorPower >= -rotateMotorMinPower) {
                    rotateMotorPower = -rotateMotorMinPower;
                }
                if (Math.abs(rotateMotorTargetPosition - rotateMotorCurrentPosition) <= 5) {

                    break;
                }

                robot.rotateMotor.setPower(rotateMotorPower);
            }
            rotateMotorPower = 0;

            robot.rotateMotor.setPower(rotateMotorPower);
            b = 1;
        }

    }
    public class shooterThread extends Thread {
        public void run() {
            if (gamepad2.right_trigger >= 0.2) {
                shooterMotorPower = 1;
            } else {
                shooterMotorPower = 0;
            }
        }

    }
}

