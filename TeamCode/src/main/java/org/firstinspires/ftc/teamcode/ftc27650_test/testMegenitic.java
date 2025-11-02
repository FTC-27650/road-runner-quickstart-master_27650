package org.firstinspires.ftc.teamcode.ftc27650_test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FTC_27650_TeleOp.MyRobotHardware_27650_TeleOp;

@TeleOp(name="测试磁性限位开关速率", group="Linear OpMode")
@Config

public class testMegenitic extends LinearOpMode {

    MyRobotHardware_27650_TeleOp robot = new MyRobotHardware_27650_TeleOp(this);

    int rotateMotorCurrentPosition = 0;
    public static int rotateMotorTargetPosition = 0;
    public static double rotatePowerStart = 0.3;
    double rotateMotorPower = 0;
    public static double rotateMotorMinPower = 0.8;

    @Override
    public void runOpMode() {
        robot.init();

        while(!robot.magnetic_in.isPressed()){
            robot.rotateMotor.setPower(rotatePowerStart);
        }
        robot.rotateMotor.setPower(0);
        robot.init();
        robot.rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()){
            rotateMotorCurrentPosition = robot.rotateMotor.getCurrentPosition();
            if(gamepad2.b){
                while(gamepad2.b){
                    if(!gamepad2.b){
                        break;
                    }
                }
                rotateMotorTargetPosition+=96;//(288/3);
            }
            rotateMotorPower =(rotateMotorTargetPosition-rotateMotorCurrentPosition)*0.01;
            if(0<rotateMotorPower && rotateMotorPower<=rotateMotorMinPower){
                rotateMotorPower = rotateMotorMinPower;
            }
            if(rotateMotorPower<0 && rotateMotorPower>=-rotateMotorMinPower){
                rotateMotorPower =- rotateMotorMinPower;
            }
            if(Math.abs(rotateMotorTargetPosition-rotateMotorCurrentPosition)<=5){
                rotateMotorPower = 0;
            }
            robot.rotateMotor.setPower(rotateMotorPower);
            telemetry.addData("旋转功率", "%4.2f",rotateMotorPower);
            telemetry.addData("旋转位置", "%7d",rotateMotorCurrentPosition);
            telemetry.addData("目标位置", "%7d",rotateMotorTargetPosition);
            telemetry.update();
        }
    }


}
