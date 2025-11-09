package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class MyRobotHardware_27650_TeleOp {

    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor br = null;
    public DcMotor bl = null;

    public DcMotor xiMotor = null;
    public DcMotor flyWheelLeft = null;
    public DcMotor flyWheelRight = null;

    public DcMotorEx rotateMotor = null;

    //public Servo rotateServo = null;
    public Servo strikerServo = null;

    public NormalizedColorSensor colorSensorFront;
    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;


    //public TouchSensor magnetic_out;
    public TouchSensor magnetic_in;
    IMU imu;
    private final LinearOpMode myOpMode;

    public MyRobotHardware_27650_TeleOp(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //端口配置
        fl = myOpMode.hardwareMap.get(DcMotor.class, "fl");//c0
        fr = myOpMode.hardwareMap.get(DcMotor.class, "fr");//c1
        br = myOpMode.hardwareMap.get(DcMotor.class, "br");//c2
        bl = myOpMode.hardwareMap.get(DcMotor.class, "bl");//c3


        xiMotor = myOpMode.hardwareMap.get(DcMotor.class, "xm");//e2

        flyWheelLeft = myOpMode.hardwareMap.get(DcMotor.class, "fwl");//e0
        flyWheelRight = myOpMode.hardwareMap.get(DcMotor.class, "fwr");//e1

        rotateMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rm");//e3

        //rotateServo = myOpMode.hardwareMap.get(Servo.class,"rs");//cs0
        strikerServo = myOpMode.hardwareMap.get(Servo.class, "ss");//cs1

        colorSensorFront = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csf");//eI1
        colorSensorLeft = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csl");//eI2
        colorSensorRight = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csr");//eI3

        //magnetic_out = myOpMode.hardwareMap.get(TouchSensor.class,"mo");//ed0
        magnetic_in = myOpMode.hardwareMap.get(TouchSensor.class, "mi");//ed6
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");//I2C BUS0

        //制动
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xiMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //马达方向
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        flyWheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheelRight.setDirection(DcMotorSimple.Direction.FORWARD);

        rotateMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


    }

}
