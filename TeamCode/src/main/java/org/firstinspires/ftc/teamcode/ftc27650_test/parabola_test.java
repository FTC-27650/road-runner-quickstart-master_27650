package org.firstinspires.ftc.teamcode.ftc27650_test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "parabola_test(抛物线)")
//@Disabled


public class parabola_test extends LinearOpMode {

    public DcMotorEx motor1 = null;
    public DcMotorEx motor2 = null;
    public Servo servo1 = null;


    double power = 0.8;
    public static double h1 = 0.65;//车高unit:m
    public static double h2 = 1.1;//塔高unit:m
    public static double k = 15;//电机系数
    public static double k1 = 0.1;//舵机系数
    double v = (power * k);//速度（电机功率*系数k）unit:(m/s)
    double g = 9.8;//重力加速度unit:(m/s*s)
    public static double d = 2.1;//水平距离（h1与h2）unit:m




    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, "m1");
        motor2 = hardwareMap.get(DcMotorEx.class, "m2");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            double discriminant = (v * v * v * v) - g * d * (2 * v * v * (h2 - h1) + g * d); // 判别式



            if (discriminant >= 0) {
                double sqrtDiscriminant = Math.sqrt(discriminant);
                double tanTheta1 = ((v * v) + sqrtDiscriminant) / (g * d);//高抛角
                double tanTheta2 = ((v * v) - sqrtDiscriminant) / (g * d);//低抛角
                double theta1 = Math.atan(tanTheta1);
                double theta2 = Math.atan(tanTheta2);


                telemetry.addData("theta1 (弧度)", "%4.4f", Math.atan(theta1));
                telemetry.addData("theta2 (弧度)", "%4.4f", Math.atan(theta2));
                telemetry.addData("theta1 (角度,高抛角)", "%4.4f", Math.toDegrees(theta1));
                telemetry.addData("theta1 (角度,低抛角)", "%4.4f", Math.toDegrees(theta2));
                telemetry.addData("discriminant(判定式)", "%4.4f", discriminant);


            } else {
                telemetry.addData("discriminant(判定式无解)", "%4.4f", discriminant);
            }


            telemetry.addData("k", "%8.8f", k);
            telemetry.addData("k1", "%8.8f", k1);
            telemetry.addData("servoPosition", "%4.4f", servo1.getPosition());



            telemetry.update();


        }

    }
}
