package org.firstinspires.ftc.teamcode.FTC_27650;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "挂高杆_4_新", group = "Autonomous")
//@Disabled
public class RED_gaoGan_4_new extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware_27650_new_auto robot = new RobotHardware_27650_new_auto(this);

    mianThread mianThread = new mianThread();

    slider_set_position slider_set_position = new slider_set_position();
    retract_set_position retract_set_position_1 = new retract_set_position();
    retract_set_position_2 retract_set_position_2 = new retract_set_position_2();
    retract_set_position_3 retract_set_position_3 = new retract_set_position_3();
    collect_motor collect_motor_xi = new collect_motor(1,1.5);
    collect_motor collect_motor_xi_2 = new collect_motor(1,1.2);
    collect_motor collect_motor_xi_3 = new collect_motor(1,1.35);
    collect_motor collect_motor_tu = new collect_motor(-1,1.2);
    collect_motor collect_motor_tu_2 = new collect_motor(-1,1.2);
    collect_motor collect_motor_tu_3 = new collect_motor(-1,1.3);

    //滑轨变量
    double sliderPower = 0;
    double sliderTargetPosition = 0;

    double retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
    double retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29

    double forward_arm_servo_position = 0.225;
    double put_servo_position = 0.52;//推出 0.34      收回 0.52
    double back_arm_servo_position = 0.22;  // 落下准备:0.22抬起放：0.55
    double claw_servo_position = 0.34;//OPEN:0.62  CLOSE:0.34

    @Override
    public void runOpMode() {

        robot.init();
        setServo_init_Position();

        sleep(200);
        runtime.reset();
        while(!robot.magnetic.isPressed() && runtime.seconds() <= 2){
            robot.slider0.setPower(-0.8);
            robot.slider2.setPower(-0.8);
            robot.slider3.setPower(-0.8);
        }
        robot.slider0.setPower(0);
        robot.slider2.setPower(0);
        robot.slider3.setPower(0);
        robot.init();

        // 以特定姿势实例化您的 MecanumDrive
        Pose2d initialPose = new Pose2d(8.858, -61.81, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // actionBuilder 从传递给它的驱动器步骤构建
        TrajectoryActionBuilder tab_gaoGan_1 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(8.858,-31.5));

        // actionBuilder 从传递给它的驱动器步骤构建
        TrajectoryActionBuilder tab_xi_3 = drive.actionBuilder(new Pose2d(8.858,-31.5, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(37,-43), Math.toRadians(63)) //吸1
                .waitSeconds(0.5)
                .strafeToLinearHeading(new Vector2d(39,-45), Math.toRadians(315)) //吐1
                .waitSeconds(0.4)
                .strafeToLinearHeading(new Vector2d(46,-43), Math.toRadians(65))//吸2
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(41,-45), Math.toRadians(310)); //吐2

        TrajectoryActionBuilder tab_jia_2 = drive.actionBuilder(new Pose2d(41,-45, Math.toRadians(310)))
                .strafeToLinearHeading(new Vector2d(37,-52.5), Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(37,-61));

        TrajectoryActionBuilder tab_gaoGan_2 = drive.actionBuilder(new Pose2d(37,-61, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(0,-40), Math.toRadians(280))
                .strafeToConstantHeading(new Vector2d(0,-34.5));

        TrajectoryActionBuilder tab_jia_3 = drive.actionBuilder(new Pose2d(0,-34.5, Math.toRadians(280)))
                .strafeToLinearHeading(new Vector2d(36,-50), Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(36,-61));

        TrajectoryActionBuilder tab_gaoGan_3 = drive.actionBuilder(new Pose2d(36,-61, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(-4,-40), Math.toRadians(280))
                .strafeToConstantHeading(new Vector2d(-4,-35));

        TrajectoryActionBuilder tab_jia_4 = drive.actionBuilder(new Pose2d(-4,-35, Math.toRadians(280)))
                .strafeToLinearHeading(new Vector2d(36,-50), Math.toRadians(100))
                .strafeToConstantHeading(new Vector2d(36,-61));

        TrajectoryActionBuilder tab_gaoGan_4 = drive.actionBuilder(new Pose2d(36,-61, Math.toRadians(100)))
                .strafeToLinearHeading(new Vector2d(-8,-40), Math.toRadians(280))
                .strafeToConstantHeading(new Vector2d(-8,-35));

        TrajectoryActionBuilder tab_tingKao = drive.actionBuilder(new Pose2d(-8,-34.5, Math.toRadians(280)))
                .strafeToLinearHeading(new Vector2d(37,-61), Math.toRadians(100));

        telemetry.addData("初始化", "完毕");
        telemetry.update();

        waitForStart();

        // 我们现在处于运行时！我们总是添加以下内容，以便在必要时能够停止机器人。
        if (isStopRequested()) return;
        slider_set_position.start();

        //初始挂第一个
        sliderTargetPosition = 1450;
        runBlocking(new SequentialAction(tab_gaoGan_1.build()));
        sliderTargetPosition = 1050;
        sleep(400);
        robot.claw_servo.setPosition(0.62);//打开夹子
        sleep(300);

        //吸3个砖块
        mianThread.start();
        runBlocking(new SequentialAction(tab_xi_3.build()));

        //夹2
        runBlocking(new SequentialAction(tab_jia_2.build()));
        robot.claw_servo.setPosition(0.34); //夹 0.34 松开 0.62
        sleep(300);
        sliderTargetPosition = 1600;
        sleep(100);

        //挂第2个
        sliderTargetPosition = 1450;
        runBlocking(new SequentialAction(tab_gaoGan_2.build()));
        sliderTargetPosition = 1000;
        sleep(400);
        robot.claw_servo.setPosition(0.62);//打开夹子
        sleep(300);

        //夹3
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_jia_3.build()));
        robot.claw_servo.setPosition(0.34); //夹 0.34 松开 0.62
        sleep(300);
        sliderTargetPosition = 1600;
        sleep(100);

        //挂第3个
        sliderTargetPosition = 1450;
        runBlocking(new SequentialAction(tab_gaoGan_3.build()));
        sliderTargetPosition = 1000;
        sleep(400);
        robot.claw_servo.setPosition(0.62);//打开夹子
        sleep(300);

        //夹4
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_jia_4.build()));
        robot.claw_servo.setPosition(0.34); //夹 0.34 松开 0.62
        sleep(300);
        sliderTargetPosition = 1600;
        sleep(100);

        //挂第4个
        sliderTargetPosition = 1450;
        runBlocking(new SequentialAction(tab_gaoGan_4.build()));
        sliderTargetPosition = 1000;
        sleep(400);
        robot.claw_servo.setPosition(0.62);//打开夹子
        sleep(300);

        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_tingKao.build()));
        sleep(2000);

    }
    public class mianThread extends Thread{
        public void run(){
            try {
                //sleep(500);

                sliderTargetPosition = 0;
                robot.forward_arm_servo.setPosition(0.68);
                sleep(500);

                collect_motor_xi.start(); // xi 1
                retract_set_position_1.start();
                sleep(2200);
                collect_motor_tu.start();
                sleep(100);
                retract_left_servo_position = 0.5;
                retract_right_servo_position = 0.5;
                robot.retract_left_servo.setPosition(0.5);
                robot.retract_right_servo.setPosition(0.5);
                sleep(1500);

                collect_motor_xi_2.start(); //xi 2
                retract_set_position_2.start();
                sleep(1400);
                collect_motor_tu_2.start();
                sleep(500);
                retract_left_servo_position = 0.5;
                retract_right_servo_position = 0.5;
                robot.retract_left_servo.setPosition(0.5);
                robot.retract_right_servo.setPosition(0.5);
                sleep(1300);
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                robot.retract_left_servo.setPosition(0.71);
                robot.retract_right_servo.setPosition(0.29);
                sleep(2000);

            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //设置舵机初始化位置
    public void setServo_init_Position(){
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        robot.forward_arm_servo.setPosition(forward_arm_servo_position);
        robot.put_servo.setPosition(put_servo_position);
        robot.back_arm_servo.setPosition(back_arm_servo_position);
        robot.claw_servo.setPosition(claw_servo_position);
    }
    //设置竖直滑轨位置线程
    public class slider_set_position extends Thread{

        public void run(){
            try {

                while(opModeIsActive()){

                    if(robot.magnetic.isPressed()){
                        robot.slider0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        robot.slider3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

                        robot.slider0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slider2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slider3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                        //slider_power = 0;
                    }
                    sliderPower = (sliderTargetPosition - robot.slider3.getCurrentPosition())*0.004;

                    robot.slider0.setPower(sliderPower);
                    robot.slider2.setPower(sliderPower);
                    robot.slider3.setPower(sliderPower);

                    sleep(10);
                }

            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //推动底部滑轨
    public class retract_set_position extends Thread{

        public void run(){
            try {

                runtime.seconds();
                double retract_servo_speed = 0.003;
                while(opModeIsActive() ){
                    retract_left_servo_position -= retract_servo_speed;
                    retract_right_servo_position += retract_servo_speed;
                    if(retract_left_servo_position<=0.25){
                        retract_left_servo_position = 0.25;
                    }
                    if(retract_right_servo_position>=0.65){
                        retract_right_servo_position = 0.65;
                    }

                    robot.retract_left_servo.setPosition(retract_left_servo_position);
                    robot.retract_right_servo.setPosition(retract_right_servo_position);

                   if(retract_left_servo_position <= 0.25 || retract_right_servo_position >=0.65){
                       sleep(10);
                       break;
                   }
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public class retract_set_position_2 extends Thread{

        public void run(){
            try {

                runtime.seconds();
                double retract_servo_speed = 0.003;
                while(opModeIsActive() ){
                    retract_left_servo_position -= retract_servo_speed;
                    retract_right_servo_position += retract_servo_speed;
                    if(retract_left_servo_position<=0.35){
                        retract_left_servo_position = 0.35;
                    }
                    if(retract_right_servo_position>=0.65){
                        retract_right_servo_position = 0.65;
                    }

                    robot.retract_left_servo.setPosition(retract_left_servo_position);
                    robot.retract_right_servo.setPosition(retract_right_servo_position);

                    if(retract_left_servo_position <= 0.35 || retract_right_servo_position >=0.65){
                        sleep(10);
                        break;
                    }
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    public class retract_set_position_3 extends Thread{

        public void run(){
            try {

                runtime.seconds();
                double retract_servo_speed = 0.002;
                while(opModeIsActive() ){
                    retract_left_servo_position -= retract_servo_speed;
                    retract_right_servo_position += retract_servo_speed;
                    if(retract_left_servo_position<=0.4){
                        retract_left_servo_position = 0.4;
                    }
                    if(retract_right_servo_position>=0.6){
                        retract_right_servo_position = 0.6;
                    }

                    robot.retract_left_servo.setPosition(retract_left_servo_position);
                    robot.retract_right_servo.setPosition(retract_right_servo_position);

                    if(retract_left_servo_position <= 0.4 || retract_right_servo_position >=0.6){
                        sleep(10);
                        break;
                    }
                }
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //吸取马达线程
    public class collect_motor extends Thread{
        double power;
        double time;
        public collect_motor (double power,double time){
            this.power = power;
            this.time = time;
        }
        public void run(){
            runtime.reset();
            while(opModeIsActive() && runtime.seconds()<=time){
                robot.collect_motor.setPower(power);
            }
            robot.collect_motor.setPower(0);
        }
    }
    //后手臂抬起
    public void back_arm(){
        runtime.reset();
        double back_arm_servo_speed = 0.005;
        back_arm_servo_position = 0.23;
        while(opModeIsActive()&&runtime.seconds()<=1200){

            back_arm_servo_position+=back_arm_servo_speed;
            if(back_arm_servo_position>=0.55){
                back_arm_servo_position=0.55;
                break;
            }
            robot.back_arm_servo.setPosition(back_arm_servo_position);
            if(back_arm_servo_position>=0.55){
                //back_arm_servo_position=0.55;
                break;
            }
        }
    }

}
