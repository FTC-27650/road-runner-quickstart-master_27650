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
@Autonomous(name = "投篮5", group = "Autonomous")
//@Disabled
public class RED_touLan5_New extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware_27650_new_auto robot = new RobotHardware_27650_new_auto(this);
    slider_set_position slider_set_position = new slider_set_position();
    retract_set_position retract_set_position_1 = new retract_set_position();
    collect_motor collect_motor_xi = new collect_motor(1,1.5);
    collect_motor collect_motor_tu = new collect_motor(-0.7,1.2);

    //滑轨变量
    double sliderPower = 0;
    double sliderTargetPosition = 0;

    double retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
    double retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
    double retract_servo_speed = 0.0035;
    double forward_arm_servo_position = 0.225;
    double put_servo_position = 0.52;//推出 0.34      收回 0.52
    double back_arm_servo_position = 0.19;  // 落下准备:0.22抬起放：0.55
    double claw_servo_position = 0.62;//OPEN:0.62  CLOSE:0.34

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
        Pose2d initialPose = new Pose2d(-35.42, -61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // actionBuilder 从传递给它的驱动器步骤构建
        TrajectoryActionBuilder tab_GAOKUANG_0 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-56.5,-54), Math.toRadians(43));

        TrajectoryActionBuilder tab_YB_1 = drive.actionBuilder(new Pose2d(-56.5,-54, Math.toRadians(43)))
                .strafeToLinearHeading(new Vector2d(-51,-46), Math.toRadians(97));

        TrajectoryActionBuilder tab_GAOKUANG_1 = drive.actionBuilder(new Pose2d(-50.5,-46, Math.toRadians(97)))
                .strafeToLinearHeading(new Vector2d(-56.5,-54), Math.toRadians(43));

        TrajectoryActionBuilder tab_YB_2 = drive.actionBuilder(new Pose2d(-56.5,-54, Math.toRadians(43)))
                .strafeToLinearHeading(new Vector2d(-62,-45), Math.toRadians(97));

        TrajectoryActionBuilder tab_GAOKUANG_2 = drive.actionBuilder(new Pose2d(-61.5,-45, Math.toRadians(97)))
                .strafeToLinearHeading(new Vector2d(-56.5,-54), Math.toRadians(43));

        TrajectoryActionBuilder tab_YB_3 = drive.actionBuilder(new Pose2d(-56.5,-54, Math.toRadians(43)))
                .strafeToLinearHeading(new Vector2d(-62,-45), Math.toRadians(126));

        TrajectoryActionBuilder tab_GAOKUANG_3 = drive.actionBuilder(new Pose2d(-62,-45, Math.toRadians(127)))
                .strafeToLinearHeading(new Vector2d(-56.5,-54), Math.toRadians(40));

        TrajectoryActionBuilder tab_YB_4 = drive.actionBuilder(new Pose2d(-56.5,-54, Math.toRadians(40)))
                .strafeToLinearHeading(new Vector2d(-24.6,-58), Math.toRadians(0));

        TrajectoryActionBuilder tab_GAOKUANG_4 = drive.actionBuilder(new Pose2d(-24.6,-58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-56.5,-54), Math.toRadians(40));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-56.5,-54, Math.toRadians(40)))
                .strafeToLinearHeading(new Vector2d(-40,-8), Math.toRadians(90));

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(-40,-8, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-28,-9), Math.toRadians(180));

        telemetry.addData("初始化", "完毕");
        telemetry.update();

        waitForStart();

        // 我们现在处于运行时！我们总是添加以下内容，以便在必要时能够停止机器人。
        if (isStopRequested()) return;
        slider_set_position.start();

        sliderTargetPosition = 2100;
        runBlocking(new SequentialAction(tab_GAOKUANG_0.build()));
        back_arm();//robot.back_arm_servo.setPosition(0.55);
        sleep(400);


        //移到第一个砖块
        robot.back_arm_servo.setPosition(0.22);
        sleep(200);
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_YB_1.build()));
        //吸1
        robot.forward_arm_servo.setPosition(0.68);//0.67
        retract_set_position_1.start();
        //sleep(100);
        collect_motor_xi.start();
        sleep(1300);
        robot.forward_arm_servo.setPosition(0.2);
        retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
        retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        sleep(500);

        //放第一个砖块
        collect_motor_tu.start();
        sleep(1200);
        sliderTargetPosition = 2100;
        //robot.back_arm_servo.setPosition(0.35);
        runBlocking(new SequentialAction(tab_GAOKUANG_1.build()));
        back_arm();//robot.back_arm_servo.setPosition(0.55);
        sleep(500);

        //去第二个砖块位置
        robot.back_arm_servo.setPosition(0.22);
        sleep(200);
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_YB_2.build()));

        //吸2
        robot.forward_arm_servo.setPosition(0.68);//0.67
        retract_set_position_1.start();
        //sleep(100);
        collect_motor_xi.start();
        sleep(1300);
        robot.forward_arm_servo.setPosition(0.20);
        retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
        retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        sleep(500);

        //放第二个砖块
        collect_motor_tu.start();
        sleep(1200);
        sliderTargetPosition = 2100;
        //robot.back_arm_servo.setPosition(0.35);
        runBlocking(new SequentialAction(tab_GAOKUANG_2.build()));
        back_arm();//robot.back_arm_servo.setPosition(0.55);
        sleep(500);

        //去第三个砖块位置
        robot.back_arm_servo.setPosition(0.22);
        sleep(200);
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_YB_3.build()));

        //吸3
        robot.forward_arm_servo.setPosition(0.68);//0.67
        retract_set_position_1.start();
        //sleep(100);
        collect_motor_xi.start();
        sleep(1300);
        robot.forward_arm_servo.setPosition(0.20);
        retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
        retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        sleep(500);

        //放第三个砖块
        collect_motor_tu.start();
        sleep(1200);
        sliderTargetPosition = 2100;
        //robot.back_arm_servo.setPosition(0.35);
        runBlocking(new SequentialAction(tab_GAOKUANG_3.build()));
        back_arm();//robot.back_arm_servo.setPosition(0.55);
        sleep(500);

        //去第四个砖块位置
        robot.back_arm_servo.setPosition(0.22);
        sleep(200);
        sliderTargetPosition = 0;
        runBlocking(new SequentialAction(tab_YB_4.build()));

        //吸4
        robot.forward_arm_servo.setPosition(0.68);//0.67
        retract_set_position_1.start();
        //sleep(100);
        collect_motor_xi.start();
        sleep(1300);
        robot.forward_arm_servo.setPosition(0.20);
        retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
        retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        sleep(500);

        //放第四个砖块
        collect_motor_tu.start();
        sleep(1200);
        sliderTargetPosition = 2100;
        runBlocking(new SequentialAction(tab_GAOKUANG_4.build()));
        back_arm();
        sleep(500);

        runBlocking(new SequentialAction(tab6.build()));
        sliderTargetPosition = 0;
        robot.back_arm_servo.setPosition(0.22);//0.65
        //runBlocking(new SequentialAction(tab7.build()));
        sleep(2500);

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
                    sliderPower = (sliderTargetPosition - robot.slider3.getCurrentPosition())*0.0085;

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
                while(opModeIsActive() && runtime.seconds()<=time){
                    retract_left_servo_position -= retract_servo_speed;
                    retract_right_servo_position += retract_servo_speed;
                    if(retract_left_servo_position<=0.29){
                        retract_left_servo_position = 0.29;
                    }
                    if(retract_right_servo_position>=0.71){
                        retract_right_servo_position = 0.71;
                    }

                    robot.retract_left_servo.setPosition(retract_left_servo_position);
                    robot.retract_right_servo.setPosition(retract_right_servo_position);

                   if(retract_left_servo_position <= 0.28 || retract_right_servo_position >=0.70){
                       sleep(100);
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
        back_arm_servo_position = 0.22;
        while(opModeIsActive()&&runtime.seconds()<=1200){

            back_arm_servo_position+=back_arm_servo_speed;
            if(back_arm_servo_position>=0.65){
                back_arm_servo_position=0.65;
                break;
            }
            robot.back_arm_servo.setPosition(back_arm_servo_position);
            if(back_arm_servo_position>=0.65){
                //back_arm_servo_position=0.55;
                break;
            }
        }
    }

}
