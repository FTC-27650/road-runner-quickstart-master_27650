package org.firstinspires.ftc.teamcode.ftc_test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "舵机测试")
@Config
//@Disabled
public class servoTest extends LinearOpMode {

    public static double s0Speed = 0.0001;
    Servo s0 = null;
    double s0Position = 0.5;
    Servo s1 = null;
    double s1Position = 0.5;
    double s1Speed = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        s0 = hardwareMap.get(Servo.class, "s0");
        s0.setPosition(s0Position);
        s1 = hardwareMap.get(Servo.class, "s1");
        s1.setPosition(s1Position);
        telemetry.addData("初始化：", "完毕");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                s0Position = Math.min(s0Position + s0Speed, 1);
                s1Position = Math.min(s1Position + s1Speed, 1);
            }
            if (gamepad1.x) {
                s0Position = 0.5;
                s1Position = 0.5;
            }
            if (gamepad1.a) {
                s0Position = Math.max(s0Position - s0Speed, 0);
                s1Position = Math.max(s1Position - s1Speed, 0);
            }
            s0.setPosition(s0Position);
            s1.setPosition(s1Position);
            telemetry.addData("s0Position", "%f", s0Position);
            telemetry.addData("s1Position", "%f", s1Position);
            telemetry.update();
        }
    }
}
