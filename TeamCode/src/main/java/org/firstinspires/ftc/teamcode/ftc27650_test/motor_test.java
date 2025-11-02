package org.firstinspires.ftc.teamcode.ftc27650_test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "motor_test(电机测试)")
public class motor_test extends LinearOpMode {

    public DcMotorEx m0 = null;
    public DcMotorEx m1 = null;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        m0 = hardwareMap.get(DcMotorEx.class, "m0");
        m1 = hardwareMap.get(DcMotorEx.class, "m1");
        m0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m0.setDirection(DcMotorEx.Direction.REVERSE);
        m1.setDirection(DcMotorEx.Direction.FORWARD);
        m0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            m0.setVelocity(5000);
            m1.setVelocity(5000);
            telemetry.addData("转速m0", m0.getVelocity());
            telemetry.addData("转速m1", m1.getVelocity());
            telemetry.update();
        }
    }
}
