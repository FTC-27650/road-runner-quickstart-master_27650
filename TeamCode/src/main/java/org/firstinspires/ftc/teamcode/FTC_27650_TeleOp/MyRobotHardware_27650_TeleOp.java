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
    public DcMotorEx flyWheelLeft = null;
    public DcMotorEx flyWheelRight = null;

    public DcMotorEx rotateMotor = null;

    public Servo strikerServo = null;

    public Servo angleServo = null;

    public NormalizedColorSensor colorSensorFront;
    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;

    //public TouchSensor magnetic_out;
    public TouchSensor magnetic_in;

    //RevBlinkinLedDriver blinkinLedDriver;

    IMU imu;

    private LinearOpMode myOpMode = null;

    public MyRobotHardware_27650_TeleOp(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //端口配置
        fl = myOpMode.hardwareMap.get(DcMotor.class, "fl");//c0
        fr = myOpMode.hardwareMap.get(DcMotor.class, "fr");//c1
        br = myOpMode.hardwareMap.get(DcMotor.class, "br");//c2
        bl = myOpMode.hardwareMap.get(DcMotor.class, "bl");//c3

        flyWheelLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fwl");//e0
        flyWheelRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fwr");//e1

        xiMotor = myOpMode.hardwareMap.get(DcMotor.class, "xm");//e2

        rotateMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rm");//e3

        strikerServo = myOpMode.hardwareMap.get(Servo.class, "ss");//es0

        angleServo = myOpMode.hardwareMap.get(Servo.class, "as");//es5

        colorSensorFront = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csf");//I2C 1
        colorSensorLeft = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csl");//I2C  2
        colorSensorRight = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csr");//I2C 3

        //magnetic_out = myOpMode.hardwareMap.get(TouchSensor.class,"mo");//ed0
        magnetic_in = myOpMode.hardwareMap.get(TouchSensor.class, "mi");//cd6

        //blinkinLedDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");//eservo0

        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // c I2C 0

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

        xiMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //((SwitchableLight)colorSensorLeft).enableLight(true);
        //((SwitchableLight)colorSensorRight).enableLight(true);
        //((SwitchableLight)colorSensorFront).enableLight(true);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        flyWheelLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void rotateMotorEncoderRest() {
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
