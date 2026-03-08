package org.firstinspires.ftc.teamcode.FTC_27650_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="手动程序27650-刑总", group="Linear OpMode")

public class Manual_Control_27650 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware_27650_new robot = new RobotHardware_27650_new(this);
    //******************************多线程******************************//
    down_prepare_xi                        down_prepare_xi = new down_prepare_xi();
    down_prepare_tu                        down_prepare_tu = new down_prepare_tu();
    up_prepare_fang_gaoKuang      up_prepare_fang_gaoKuang = new up_prepare_fang_gaoKuang();
    up_prepare_shou_jia                up_prepare_shou_jia = new up_prepare_shou_jia();
    up_prepare_fang_gaoGan          up_prepare_fang_gaoGan = new up_prepare_fang_gaoGan();
    up_prepare_gaoGan_down          up_prepare_gaoGan_down = new up_prepare_gaoGan_down();
    up_prepare_fang_diKuang         up_prepare_fang_diKuang = new up_prepare_fang_diKuang();
    //******************************多线程using******************************//
    boolean up_prepare_shou_jia_using = true;
    boolean up_prepare_fang_gaoKuang_using = true;
    boolean up_prepare_fang_gaoGan_using = true;
    boolean up_prepare_gaoGan_down_using = true;

    //*****************************变量******************************//

    //**********底盘变量**********//
    double n = 1; int a = 2;
    boolean mecaunm_1_Using = false; boolean mecaunm_2_Using = true;
    //**********收集器电机**********//
    double collect_power = 0;
    //**********滑轨电机**********//
    double sliderPower = 0;
    double sliderTargetPosition = 0;
    //**********伸缩舵机**********//
    double retract_left_servo_position = 0.71;  // DOWN0.29    //UP0.71
    double retract_right_servo_position = 0.29; //DOWN0.71    //UP0.29
    double retract_servo_speed = 0.01;
    //**********前手臂舵机**********//
    double forward_arm_servo_position = 0.225;
    double forward_arm_servo_speed = 0.0025;
    //**********前推杆舵机**********//
    double put_servo_position = 0.52;//推出 0.34      收回 0.52
    double put_servo_speed = 0.01;
    //**********篮筐舵机**********//
    double back_arm_servo_position = 0.22;  // 落下准备:0.22抬起放：0.55
    double back_arm_servo_speed = 0.01;
    //**********夹子舵机**********//
    double claw_servo_position = 0.5;//OPEN:0.62  CLOSE:0.34
    double claw_servo_speed = 0.001;
    //**********方法using**********//
    boolean collect_using = true;
    //********************初始化参数结束********************//

    double presentVoltage = 0;


    @Override
    //**********方法**********//
    public void runOpMode() {

        //初始化//
        robot.init();
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        robot.forward_arm_servo.setPosition(forward_arm_servo_position);
        robot.put_servo.setPosition(put_servo_position);
        robot.back_arm_servo.setPosition(back_arm_servo_position);
        robot.claw_servo.setPosition(claw_servo_position);
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
        telemetry.addData("状态", "初始化完毕");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            //presentVoltage = robot.myControlHubVoltageSensor.getVoltage();

            //手柄2 左肩键  篮子下落准备收
            if(gamepad2.right_bumper){
                up_prepare_shou_jia_using=true;
                while(up_prepare_shou_jia_using && opModeIsActive()){
                    if(gamepad2.right_bumper){up_prepare_shou_jia.start();}
                    putong();
                }
            }
            //手柄2 左肩键上  篮子升高准备放高框
            if(gamepad2.left_bumper){
                up_prepare_fang_gaoKuang_using=true;
                while(up_prepare_fang_gaoKuang_using && opModeIsActive()){
                    if(gamepad2.left_bumper){up_prepare_fang_gaoKuang.start();}
                    putong();
                }
            }
            //手柄2 右肩键上 抬升准备挂高杆
            if(gamepad2.right_trigger>0.2){
                up_prepare_fang_gaoGan_using=true;
                while(up_prepare_fang_gaoGan_using && opModeIsActive()){
                    if(gamepad2.right_trigger>0.2){up_prepare_fang_gaoGan.start();}
                    putong();
                }
            }
            //手柄2 右肩键下 下降挂高杆
            if(gamepad2.left_trigger>0.2){
                up_prepare_gaoGan_down_using=true;
                while(up_prepare_gaoGan_down_using && opModeIsActive()){
                    if(gamepad2.left_trigger>0.2){up_prepare_gaoGan_down.start();}
                    putong();
                }
            }
            if(gamepad2.right_stick_button){up_prepare_fang_diKuang.start();}
            //普通方法//
            putong();
        }

    }
    //**********普通方法导入**********//
    public void putong(){
        if(gamepad1.a){down_prepare_xi.start();}
        if(gamepad1.y){down_prepare_tu.start();}
        //底盘模式切换：（手柄1）左摇杆按压切换模式1；右摇杆按压切换模式2//
        //if(gamepad1.left_stick_button){mecaunm_1_Using=true;mecaunm_2_Using=false;a=1;}
        if(gamepad1.right_stick_button){mecaunm_2_Using=true;mecaunm_1_Using=false;a=2;}
        //if(mecaunm_1_Using){mecanum_1();}
        if(mecaunm_2_Using){mecanum_2();}
        magnetic();
        slider(0.3,0.9,350,2150);//2100为极限
        if(collect_using){collect();}
        retractServoControl();
        arm_and_put_servo();
        arm_and_claw_servo();

        show();

    }
    //**********drive hub显示**********//
    public void show(){
        telemetry.addData("Z轴", "%4.4f",-robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //telemetry.addData("y/x/rx=", "%4.4f, %4.2f, %4.4f",-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
        telemetry.addData("前轮功率 left/right", "%4.4f, %4.4f",robot.fl.getPower(),robot.fr.getPower());
        telemetry.addData("后轮功率 left/right", "%4.4f, %4.4f",robot.bl.getPower(),robot.br.getPower());
        telemetry.addData("底盘模式", "%7d",a);
        telemetry.addData("滑轨功率", "%4.4f",sliderPower);
        telemetry.addData("滑轨位置 目标 / 现在", "%4.4f, %7d",sliderTargetPosition,robot.slider3.getCurrentPosition());
        telemetry.addData("伸缩位置 左/右", "%4.4f, %4.4f",retract_left_servo_position,retract_right_servo_position);
        telemetry.addData("前手臂/后篮筐位置", "%4.4f, %4.4f",forward_arm_servo_position,back_arm_servo_position);
        telemetry.addData("前推杆位置","%4.4f",put_servo_position);
        telemetry.addData("后夹子位置","%4.4f",claw_servo_position);
        telemetry.addData("主机电压", "%4.2f",presentVoltage);
        telemetry.update();
    }
    //**********底盘驱动1**********//
    
    //gamepad1.left_stick_button
    public void magnetic() {
        if(robot.magnetic.isPressed()){
            robot.slider0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.slider3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            robot.slider0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void mecanum_1(){
        double max;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x*n;


        double flPower = y + x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;
        double blPower = y - x + rx;

        max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));

        if (max > 1.0) {
            flPower  /= max;
            frPower /= max;
            blPower   /= max;
            brPower  /= max;
        }
        if(gamepad1.left_bumper){
            flPower /=2;
            frPower /=2;
            brPower /=2;
            blPower /=2;
            n=0.5;
        }
        else{
            n=1;
        }
        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.bl.setPower(blPower);
        robot.br.setPower(brPower);
    }
    //**********底盘驱动2**********/
    //gamepad1.right_stick_button
    public void mecanum_2(){

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x*n;

        if (gamepad1.start) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double flPower = (rotY + rotX + rx) / denominator;
        double blPower = (rotY - rotX + rx) / denominator;
        double frPower = (rotY - rotX - rx) / denominator;
        double brPower = (rotY + rotX - rx) / denominator;

        if(gamepad1.left_bumper){
            flPower /=2;
            frPower /=2;
            brPower /=2;
            blPower /=2;
            n=0.5;
        }
        else{
            n=1;
        }
        robot.fl.setPower(flPower);
        robot.fr.setPower(frPower);
        robot.bl.setPower(blPower);
        robot.br.setPower(brPower);
    }
    //**********滑轨控制**********//
    //UP:gamepad2.left_stick_y;DOWN:gamepad2.left_stick_y
    public void slider(double minSpeed, double maxSpeed, double position, int sliderMaxPosition){
        if(gamepad2.left_stick_y<-0.2){
            if(robot.slider3.getCurrentPosition() <= sliderMaxPosition - position){
                sliderPower = Math.min(maxSpeed,-gamepad2.left_stick_y);
            }else {
                sliderPower = minSpeed + ((sliderMaxPosition-robot.slider3.getCurrentPosition())/position)*(maxSpeed-minSpeed);
            }
            sliderTargetPosition = Math.min(robot.slider3.getCurrentPosition(),sliderMaxPosition);
        }
        else if(gamepad2.left_stick_y>0.2){
            if(robot.slider3.getCurrentPosition() >= position){
                sliderPower = Math.max(-maxSpeed,-gamepad2.left_stick_y);
            }else {
                sliderPower = -minSpeed/2 + (robot.slider3.getCurrentPosition()/position)*(minSpeed/2-maxSpeed);
            }
            sliderTargetPosition = Math.max(robot.slider3.getCurrentPosition(),0);
        }
        else{
            sliderPower = (sliderTargetPosition - robot.slider3.getCurrentPosition())*0.01;
        }

        robot.slider0.setPower(sliderPower);
        robot.slider2.setPower(sliderPower);
        robot.slider3.setPower(sliderPower);
    }
    //**********前收集器（电机）控制**********//
    //USE正转:gamepad2.y;USE反转:gamepad2.a;STOP:gamepad2.left_stick_button
    public void collect(){

        if(gamepad2.y) collect_power = 1;

        if(gamepad2.a) collect_power = -0.7;//-0.84;

        if(gamepad2.left_stick_button) collect_power = 0;

        robot.collect_motor.setPower(collect_power);
    }
    //**********前滑轨伸缩舵机控制**********//
    //伸出:gamepad1.right_stick_y;收回:gamepad1.right_stick_y
    public void retractServoControl(){
        //伸缩装置 手柄1 右摇杆 Y方向
        if(gamepad1.dpad_up){
            retract_left_servo_position += retract_servo_speed;
            retract_right_servo_position -= retract_servo_speed;
            if(retract_left_servo_position>=0.71){
                retract_left_servo_position = 0.71;
            }
            if(retract_right_servo_position<=0.29){
                retract_right_servo_position = 0.29;
            }
        }
        if(gamepad1.dpad_down){
            retract_left_servo_position -= retract_servo_speed;
            retract_right_servo_position += retract_servo_speed;
            if(retract_left_servo_position<=0.29){
                retract_left_servo_position = 0.29;
            }
            if(retract_right_servo_position>=0.71){
                retract_right_servo_position = 0.71;
            }
        }

        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);

    }
    //**********前手臂舵机和前推杆舵机控制**********//
    // 前推杆舵机:gamepad1.x前推;gamepad1.b缩回   慢前推 十字左   慢收回 十字右
    // 前手臂缓慢抬起:gamepad1.right_bumper;前手臂缓慢下落:gamepad1.right_trigger
    public void arm_and_put_servo(){

        /*前手臂缓慢抬起：手柄1 右肩键上   前手臂缓慢下落：手柄1 右肩键下
        if(gamepad1.dpad_up){
            forward_arm_servo_position -= forward_arm_servo_speed;
            if(forward_arm_servo_position<=0.2){
                forward_arm_servo_position = 0.2;
            }
        }
        if(gamepad1.dpad_down){
            forward_arm_servo_position += forward_arm_servo_speed;
            if(forward_arm_servo_position>=0.8){
                forward_arm_servo_position = 0.8;
            }
        }
         */
        if(gamepad1.right_bumper){forward_arm_servo_position = 0.225;}
        if(gamepad1.right_trigger>0.2){forward_arm_servo_position = 0.685;}

         //前推杆舵机  手柄1 x前推  b缩回
        if(gamepad1.dpad_left){
           put_servo_position -= put_servo_speed;
            if(put_servo_position<=0){
                put_servo_position = 0;
            }
        }
        if(gamepad1.dpad_right){
            put_servo_position += put_servo_speed;
            if(put_servo_position>=0.52){
                put_servo_position = 0.52;
            }
        }
        if(gamepad1.x){put_servo_position = 0.25;}
        if(gamepad1.b){put_servo_position = 0.52;}
        robot.forward_arm_servo.setPosition(forward_arm_servo_position);
        robot.put_servo.setPosition(put_servo_position);
    }

    //**********后手臂（篮子）舵机和后夹子舵机控制**********//
    // 篮筐微调:gamepad2.right_stick_y;放:gamepad2.dpad_up;收:gamepad2.dpad_down
    // 后夹子开:gamepad2.x;闭:gamepad2.b
    public void arm_and_claw_servo(){
        // 篮筐转圈  手柄2 右摇杆Y方向
        if(-gamepad2.right_stick_y>=0.2){
            back_arm_servo_position+=back_arm_servo_speed;
            if(back_arm_servo_position>=1){
                back_arm_servo_position=1;
            }
        }
        if(-gamepad2.right_stick_y<=-0.2){
            back_arm_servo_position-=back_arm_servo_speed;
            if(back_arm_servo_position<=0.14){
                back_arm_servo_position=0.14;
            }
        }
        //放 手柄2 十字上键
        if(gamepad2.dpad_up){back_arm_servo_position=0.65;}
        //收 手柄2 十字下键
        if(gamepad2.dpad_down){back_arm_servo_position=0.22;}
        //******************五、后夹子装置*************** //
        //后夹子缓慢动作  关闭：手柄2十字按钮左       打开：手柄2十字按钮右
        if(gamepad2.dpad_right){
            claw_servo_position += claw_servo_speed;
            if(claw_servo_position>=0.9){
                claw_servo_position = 0.9;
            }
        }
        if(gamepad2.dpad_left){
            claw_servo_position -= claw_servo_speed;
            if(claw_servo_position<=0.55){
                claw_servo_position = 0.55;
            }
        }
        // 后置夹子张开闭合快  x 闭合   b 张开
        if(gamepad2.x){claw_servo_position = 0.34;}
        if(gamepad2.b){claw_servo_position = 0.62;}

        robot.back_arm_servo.setPosition(back_arm_servo_position);
        robot.claw_servo.setPosition(claw_servo_position);

    }

    //**********准备吸入标本**********//
    //gamepad1.a
    public class down_prepare_xi extends Thread{
        public void run(){
            try{

                forward_arm_servo_position = 0.55;  //舵机中间位置

                sleep(300);
                retract_right_servo_position = 0.5;
                retract_left_servo_position = 0.5;
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //**********准备投放标本（到车载篮子里）**********//
    //gamepad1.y
    public class down_prepare_tu extends Thread{
        public void run(){
            try{

                forward_arm_servo_position = 0.55;
                sleep(100);
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                sleep(300);
                collect_power  =  0;
                forward_arm_servo_position=0.225;


            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //**********准备投放标本（到高框里）**********//
    //gamepad2.right_trigger
    public class up_prepare_fang_gaoKuang extends Thread{
        public void run(){

            try {
                up_prepare_fang_gaoKuang_using = true;

                back_arm_servo_position = 0.22;//
                forward_arm_servo_position = 0.225;
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                sliderTargetPosition = 0;
                collect_power = 0;
                sleep(600);
                collect_using = false;
                robot.collect_motor.setPower(-0.7);
                sleep(600);
                robot.collect_motor.setPower(0);
                collect_using = true;
                sliderTargetPosition = 2073;//2100
                //back_arm_servo_position = 0.35;
                sleep(500);
                up_prepare_fang_gaoKuang_using = false;
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //gamepad2.a
    public class up_prepare_fang_diKuang extends Thread{
        public void run(){

            try {
                up_prepare_fang_gaoKuang_using = true;

                back_arm_servo_position = 0.22;//
                forward_arm_servo_position = 0.225;
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                sliderTargetPosition = 0;
                collect_power = 0;
                sleep(600);
                collect_using = false;
                robot.collect_motor.setPower(-0.7);
                sleep(600);
                robot.collect_motor.setPower(0);
                collect_using = true;
                sliderTargetPosition = 900;//2100
                back_arm_servo_position = 0.35;
                sleep(500);
                up_prepare_fang_gaoKuang_using = false;
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //**********准备收集带有夹子标本（人玩区）**********//
    //gamepad2.right_bumper
    public class up_prepare_shou_jia extends Thread{
        public void run(){
            try{
                up_prepare_shou_jia_using = true;
                claw_servo_position = 0.34;

                sleep(500);
                sliderTargetPosition = 1460;
                sleep(1000);
                up_prepare_shou_jia_using = false;


            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //**********准备投放带有夹子的标本（到高杆上）**********//
    //gamepad2.left_bumper
    public class up_prepare_fang_gaoGan extends Thread{
        public void run(){
            try{
                up_prepare_fang_gaoGan_using = true;

                sliderTargetPosition = 1000;
                sleep(300);
                claw_servo_position = 0.62;
                sleep(300);
                sliderTargetPosition = 0;
                sleep(1000);
                up_prepare_fang_gaoGan_using = false;

            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //**********滑轨降到底（后夹子会张开）**********//
    //gamepad2.left_trigger
    public class up_prepare_gaoGan_down extends Thread{
        public void run(){
            up_prepare_gaoGan_down_using = true;
            back_arm_servo_position = 0.19;
            claw_servo_position = 0.62;
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            sliderTargetPosition = 0;
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            up_prepare_gaoGan_down_using = false;

        }
    }


}
