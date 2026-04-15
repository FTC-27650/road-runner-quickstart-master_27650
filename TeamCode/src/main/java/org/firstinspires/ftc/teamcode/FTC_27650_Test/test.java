package org.firstinspires.ftc.teamcode.FTC_27650_Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test", group = "LinearOpmode")
@Config
//@Disabled
public class test extends LinearOpMode {
    public DcMotorEx a = null;
    public Servo b = null;
    public AnalogInput steeringEncoder = null;

    public static double STEERING_CENTER = 0.5;
    public static double STEERING_RANGE = 0.5;
    public static double DEADZONE = 0.1;
    public static double POWER_SCALE = 1.0;
    public static double STEERING_SENSITIVITY = 1.0;

    public void runOpMode() {
        a = hardwareMap.get(DcMotorEx.class, "a");
        b = hardwareMap.get(Servo.class, "b");
        steeringEncoder = hardwareMap.get(AnalogInput.class, "steeringEncoder");

        a.setDirection(DcMotorEx.Direction.FORWARD);
        a.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        b.setPosition(STEERING_CENTER);

        waitForStart();

        while (opModeIsActive()) {
            double drivePower = -gamepad1.left_stick_y;
            double turnInput = gamepad1.left_stick_x;

            if (Math.abs(drivePower) < DEADZONE) {
                drivePower = 0;
            }

            if (Math.abs(turnInput) < DEADZONE) {
                turnInput = 0;
            }

            double steeringAngle = STEERING_CENTER + (turnInput * STEERING_RANGE * STEERING_SENSITIVITY);

            steeringAngle = Math.max(0, Math.min(1, steeringAngle));

            b.setPosition(steeringAngle);

            double encoderValue = steeringEncoder.getVoltage();
            double encoderPosition = encoderValue / 3.3;

            double actualPower = drivePower * POWER_SCALE;
            a.setPower(actualPower);

            telemetry.addData("驱动功率", "%.2f", actualPower);
            telemetry.addData("舵机位置", "%.3f", steeringAngle);
            telemetry.addData("编码器电压", "%.3fV", encoderValue);
            telemetry.addData("编码器归一化", "%.3f", encoderPosition);
            telemetry.addData("转向输入", "%.2f", turnInput);
            telemetry.addData("驱动输入", "%.2f", drivePower);
            telemetry.update();
        }
    }
}
