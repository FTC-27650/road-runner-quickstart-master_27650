package org.firstinspires.ftc.teamcode.FTC_27650_AUTO_NEW;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MyRobotHardware_27650_AUTO_new {

    public DcMotorEx fl = null;
    public DcMotorEx fr = null;
    public DcMotorEx br = null;
    public DcMotorEx bl = null;

    public DcMotorEx xiMotor = null;
    public DcMotorEx flyWheelLeft = null;
    public DcMotorEx flyWheelRight = null;

    public DcMotorEx rotateMotor = null;
    public DigitalChannel indexPin;

    public Servo strikerServo = null;       //servo hub 0 //down:0.186 up:0.384
    public Servo angleServo = null;         //servo hub 1 //down:0 up:1
    public Servo leftTurretServo = null;    //servo hub 2
    public Servo rightTurretServo = null;   //servo hub 3

    public NormalizedColorSensor colorSensorFront;
    public NormalizedColorSensor colorSensorLeft;
    public NormalizedColorSensor colorSensorRight;

    public TouchSensor magnetic_in;

    public DigitalChannel ledPin;
    //RevBlinkinLedDriver blinkinLedDriver;

    public Limelight3A limelight;
    IMU imu;
    public VoltageSensor batterySensor;

    private LinearOpMode myOpMode = null;
    public MyRobotHardware_27650_AUTO_new(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        //端口配置
        fl = myOpMode.hardwareMap.get(DcMotorEx.class, "fl");//c0
        fr = myOpMode.hardwareMap.get(DcMotorEx.class, "fr");//c1
        br = myOpMode.hardwareMap.get(DcMotorEx.class, "br");//c2
        bl = myOpMode.hardwareMap.get(DcMotorEx.class, "bl");//c3

        flyWheelLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "fwl");//e0
        flyWheelRight = myOpMode.hardwareMap.get(DcMotorEx.class, "fwr");//e1

        xiMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "xm");//e2

        rotateMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "rm");//e3 弹舱
        indexPin = myOpMode.hardwareMap.get(DigitalChannel.class, "ip");//ed2

        strikerServo = myOpMode.hardwareMap.get(Servo.class, "ss");      //servo hub 0  扳机
        angleServo = myOpMode.hardwareMap.get(Servo.class, "as");        //servo hub 1
        leftTurretServo = myOpMode.hardwareMap.get(Servo.class, "lrs");  //servo hub 2
        rightTurretServo = myOpMode.hardwareMap.get(Servo.class, "rrs"); //servo hub 3

        colorSensorFront = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csf");//c I2C 1
        colorSensorLeft = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csl");//c I2C  2
        colorSensorRight = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "csr");//c I2C 3

        magnetic_in = myOpMode.hardwareMap.get(TouchSensor.class, "mi");//cd6

        ledPin = myOpMode.hardwareMap.get(DigitalChannel.class, "lp");//cd2
        ledPin.setMode(DigitalChannel.Mode.OUTPUT);
        ledPin.setState(true); // 默认高电平
        //blinkinLedDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");//eservo0
        limelight = myOpMode.hardwareMap.get(Limelight3A.class, "limelight");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu"); // c I2C 0

        batterySensor = myOpMode.hardwareMap.voltageSensor.iterator().next();

        //制动
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        imu.resetYaw();

        flyWheelLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flyWheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        xiMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        xiMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void rotateMotorEncoderRest() {
        // 2. 设置编码器模式
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexPin.setMode(DigitalChannel.Mode.INPUT);
    }
}
