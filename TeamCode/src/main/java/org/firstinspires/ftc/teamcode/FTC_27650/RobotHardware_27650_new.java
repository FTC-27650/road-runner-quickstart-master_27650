package org.firstinspires.ftc.teamcode.FTC_27650;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class RobotHardware_27650_new {

    private LinearOpMode myOpMode = null;
    //底盘马达
    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx br = null;
    public DcMotorEx bl = null;
    //滑轨马达
    public DcMotorEx slider0 = null;
    public DcMotorEx slider2 = null;
    public DcMotorEx slider3 = null;
    //收集马达
    public DcMotor collect_motor = null;

    //伸缩舵机
    public Servo retract_left_servo = null;
    public Servo retract_right_servo = null;

    public Servo forward_arm_servo = null;
    public Servo put_servo = null;

    public Servo back_arm_servo = null; //后拾取
    public Servo claw_servo = null;

    public IMU imu;
    public TouchSensor magnetic;
    public VoltageSensor myControlHubVoltageSensor;

    public RobotHardware_27650_new(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(){
        //配置端口
        fl = myOpMode.hardwareMap.get(DcMotorEx.class,"fl"); // c0
        fr = myOpMode.hardwareMap.get(DcMotorEx.class,"fr"); // c1
        br = myOpMode.hardwareMap.get(DcMotorEx.class,"br"); // c2
        bl = myOpMode.hardwareMap.get(DcMotorEx.class,"bl"); // c3

        collect_motor = myOpMode.hardwareMap.get(DcMotor.class, "cm");//e1

        slider0 = myOpMode.hardwareMap.get(DcMotorEx.class,"s0"); // e0
        slider2 = myOpMode.hardwareMap.get(DcMotorEx.class,"s2"); // e2
        slider3 = myOpMode.hardwareMap.get(DcMotorEx.class,"s3"); // e3

        retract_left_servo = myOpMode.hardwareMap.get(Servo.class,"rls"); // c0
        retract_right_servo = myOpMode.hardwareMap.get(Servo.class,"rrs"); // e0

        forward_arm_servo = myOpMode.hardwareMap.get(Servo.class,"fas"); // e2
        put_servo = myOpMode.hardwareMap.get(Servo.class,"ps"); // e1

        back_arm_servo = myOpMode.hardwareMap.get(Servo.class, "bas");//c5
        claw_servo = myOpMode.hardwareMap.get(Servo.class, "cs");//c4

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");       // control
        magnetic = myOpMode.hardwareMap.get(TouchSensor.class, "magnetic");//c d 6
        //myControlHubVoltageSensor = myOpMode.hardwareMap.get(VoltageSensor.class,"mcv");

        //设置方向
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        slider0.setDirection(DcMotorSimple.Direction.FORWARD);
        slider2.setDirection(DcMotorSimple.Direction.FORWARD);
        slider3.setDirection(DcMotorSimple.Direction.FORWARD);

        //制动
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collect_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //重置马达编码器
        mecanumEncoderRest();
        sliderEncoderRest();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

    }
    public void mecanumEncoderRest(){
        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void sliderEncoderRest(){
        slider0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slider3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slider0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slider2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slider3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
