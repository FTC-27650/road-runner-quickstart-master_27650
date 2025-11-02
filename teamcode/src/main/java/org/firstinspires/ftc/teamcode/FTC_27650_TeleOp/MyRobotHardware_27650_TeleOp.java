package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MyRobotHardware_27650_TeleOp {

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx br = null;
    public DcMotorEx bl = null;

    public DcMotorEx rotateMotor = null;

    //public Servo rotateServo = null;
    public Servo strikerServo = null;

    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;

    public TouchSensor magnetic_out;
    public TouchSensor magnetic_in;

    private LinearOpMode myOpMode = null;
    public MyRobotHardware_27650_TeleOp (LinearOpMode opmode) {
       myOpMode = opmode;
    }

    public void init(){
        //端口配置
        fl = myOpMode.hardwareMap.get(DcMotorEx.class,"fl");//c0
        fr = myOpMode.hardwareMap.get(DcMotorEx.class,"fr");//c1
        br = myOpMode.hardwareMap.get(DcMotorEx.class,"br");//c2
        bl = myOpMode.hardwareMap.get(DcMotorEx.class,"bl");//c3

        rotateMotor = myOpMode.hardwareMap.get(DcMotorEx.class,"rm");//e0

        //rotateServo = myOpMode.hardwareMap.get(Servo.class,"rs");//cs0
        strikerServo = myOpMode.hardwareMap.get(Servo.class,"ss");//cs1

        colorSensorLeft = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csl");//cI1
        colorSensorRight = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csr");//cI2

        magnetic_out = myOpMode.hardwareMap.get(TouchSensor.class,"mo");//cd0
        magnetic_in = myOpMode.hardwareMap.get(TouchSensor.class,"mi");//cd2

        //制动
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //马达方向
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        rotateMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //((SwitchableLight)colorSensorLeft).enableLight(true);
        //((SwitchableLight)colorSensorRight).enableLight(true);

    }

}
