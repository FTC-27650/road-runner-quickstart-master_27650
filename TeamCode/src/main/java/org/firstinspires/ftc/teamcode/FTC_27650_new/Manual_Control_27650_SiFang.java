package org.firstinspires.ftc.teamcode.FTC_27650_new;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="手动程序27650——四方", group="Linear OpMode")
@Disabled
public class Manual_Control_27650_SiFang extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RobotHardware_27650_new robot = new RobotHardware_27650_new(this);

    down_prepare_xi down_prepare_xi = new down_prepare_xi();
    down_prepare_tu down_prepare_tu = new down_prepare_tu();
    up_prepare_fang_gaoKuang up_prepare_fang_gaoKuang = new up_prepare_fang_gaoKuang();
    up_prepare_gaoGan_down up_prepare_gaoGan_down = new up_prepare_gaoGan_down();
    up_prepare_shou_jia up_prepare_shou_jia = new up_prepare_shou_jia();
    up_prepare_fang_gaoGan up_prepare_fang_gaoGan = new up_prepare_fang_gaoGan();

    // 底盘变量
    double n = 1; int a = 2;
    boolean mecaunm_1_Using = false; boolean mecaunm_2_Using = true;

    // 收集马达变量
    double collect_power = 0; boolean collect_using = true;

    //滑轨变量
    double sliderPower = 0; double sliderTargetPosition = 0;

    //伸缩舵机变量
    double retract_left_servo_position = 0.71;  // 0.29    //0.71
    double retract_right_servo_position = 0.29; // 0.71    //0.29
    double retract_servo_speed = 0.005;

    //前手臂舵机 + //前推杆舵机
    double forward_arm_servo_position = 0.33; //落下收集0.8  抬起准备投 0.24
    double forward_arm_servo_speed = 0.0025;
    double put_servo_position = 0.52;   //推出 0.34      收回 0.52
    double put_servo_speed = 0.01;

    //后手臂舵机 + 夹子舵机
    double back_arm_servo_position = 0.22;  // 缩回0.22  投放0.55
    double back_arm_servo_speed = 0.001;
    double claw_servo_position = 0.5;
    double claw_servo_speed = 0.001;
    @Override
    public void runOpMode() {

        robot.init();
        //初始化伸缩舵机位置
        robot.retract_left_servo.setPosition(retract_left_servo_position);
        robot.retract_right_servo.setPosition(retract_right_servo_position);
        robot.forward_arm_servo.setPosition(forward_arm_servo_position);
        robot.put_servo.setPosition(put_servo_position);
        robot.back_arm_servo.setPosition(back_arm_servo_position);
        robot.claw_servo.setPosition(claw_servo_position);
        sleep(200);

        runtime.reset();
        while(!robot.magnetic.isPressed() && runtime.seconds() <= 3){
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
            //底盘手按下 a 准备吸  进中间准备按右肩键下
            if(gamepad1.a){down_prepare_xi.start();}
            //底盘手按下 y 准备吐  出中间准备按 右肩键上
            if(gamepad1.y){down_prepare_tu.start();}
            //准备投放标本（到高框里）左肩键上
            if(gamepad2.left_bumper){up_prepare_fang_gaoKuang.start();}
            //滑轨降到底（后夹子会张开） 左肩键下
            if(gamepad2.left_trigger>0.1){up_prepare_gaoGan_down.start();}
            //准备收集带有夹子标本（人玩区） 夹住后会上升 右肩键上
            if(gamepad2.right_bumper){up_prepare_shou_jia.start();}
            //gamepad2.right_trigger  准备投放带有夹子的标本（到高杆上）
            if(gamepad2.right_trigger>0.1){up_prepare_fang_gaoGan.start();}


            //底盘模式切换    手柄1   左摇杆按压切换模式1   右摇杆按压切换模式2
            if(gamepad1.left_stick_button){mecaunm_1_Using=true;mecaunm_2_Using=false;a=1;}
            if(gamepad1.right_stick_button){mecaunm_2_Using=true;mecaunm_1_Using=false;a=2;}
            if(mecaunm_1_Using){mecanum_1();}
            if(mecaunm_2_Using){mecanum_2();}

            magnetic();//重置滑轨
            slider(0.45,0.9,300,1900);//2000为极限
            if(collect_using){collect();}

            retractServoControl();
            arm_and_put_servo();
            arm_and_claw_servo();

            show();
        }

    }

    public void show(){
        telemetry.addData("底盘模式", "%7d",a);
        telemetry.addData("滑轨功率", "%4.2f",sliderPower);
        telemetry.addData("滑轨位置 目标 / 现在", "%4.2f, %7d",sliderTargetPosition,robot.slider3.getCurrentPosition());
        telemetry.addData("伸缩位置 左/右", "%4.2f, %4.2f",retract_left_servo_position,retract_right_servo_position);
        telemetry.addData("前手臂/后手臂", "%4.2f, %4.2f",forward_arm_servo_position,back_arm_servo_position);
        telemetry.update();
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
    public void magnetic() {
        if(robot.magnetic.isPressed()){
            robot.slider0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.slider2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.slider3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            robot.slider0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            robot.slider3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            //slider_power = 0;
        }
    }
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
                sliderPower = -minSpeed/2 + (robot.slider3.getCurrentPosition()/position)*(minSpeed-maxSpeed)/2;
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
    public void collect(){

        if(gamepad2.y){
            collect_power = 1;
        }
        if(gamepad2.a){
            collect_power = -0.8;
        }
        if(gamepad2.left_stick_button){
            collect_power = 0;
        }
        robot.collect_motor.setPower(collect_power);
    }
    //伸缩舵机控制
    public void retractServoControl(){
        //伸缩装置 手柄1 右摇杆 Y方向
        if(-gamepad1.right_stick_y<-0.1){
            retract_left_servo_position += Math.abs(gamepad1.right_stick_y)*retract_servo_speed;
            retract_right_servo_position -= Math.abs(gamepad1.right_stick_y)*retract_servo_speed;
            if(retract_left_servo_position>=0.71){
                retract_left_servo_position = 0.71;
            }
            if(retract_right_servo_position<=0.29){
                retract_right_servo_position = 0.29;
            }
        }
        if(-gamepad1.right_stick_y>0.1){
            retract_left_servo_position -= Math.abs(gamepad1.right_stick_y)*retract_servo_speed;
            retract_right_servo_position += Math.abs(gamepad1.right_stick_y)*retract_servo_speed;
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
    //前手臂+推杆
    public void arm_and_put_servo(){

        //前手臂缓慢抬起：手柄1 十字上   前手臂缓慢下落：手柄1 十字下
        if(gamepad1.dpad_up){
            forward_arm_servo_position -= forward_arm_servo_speed;
            if(forward_arm_servo_position<=0.24){
                forward_arm_servo_position = 0.24;
            }
        }
        if(gamepad1.dpad_down){
            forward_arm_servo_position += forward_arm_servo_speed;
            if(forward_arm_servo_position>=0.8){
                forward_arm_servo_position = 0.8;
            }
        }
        // 按下右肩键下准备吸    按下右肩键上准备吐
        if(gamepad1.right_trigger>=0.1){forward_arm_servo_position = 0.68;}
        if(gamepad1.right_bumper){forward_arm_servo_position = 0.24;}

        // 前推杆舵机  手柄1 y前推  a缩回
        if(gamepad1.dpad_right){
            put_servo_position -= put_servo_speed;
            if(put_servo_position<=0.34){
                put_servo_position = 0.34;
            }
        }
        if(gamepad1.dpad_left){
            put_servo_position += put_servo_speed;
            if(put_servo_position>=0.52){
                put_servo_position = 0.52;
            }
        }
        if(gamepad1.b){put_servo_position = 0.34;}
        if(gamepad1.x){put_servo_position = 0.52;}

        robot.forward_arm_servo.setPosition(forward_arm_servo_position);
        robot.put_servo.setPosition(put_servo_position);
    }
    //后手臂+夹子
    public void arm_and_claw_servo(){
        // 篮筐转圈  手柄2 右摇杆Y方向
        if(-gamepad2.right_stick_y>=0.2){
            back_arm_servo_position+=back_arm_servo_speed;
            if(back_arm_servo_position>=0.7){
                back_arm_servo_position=0.7;
            }
        }
        if(-gamepad2.right_stick_y<=-0.2){
            back_arm_servo_position-=back_arm_servo_speed;
            if(back_arm_servo_position<=0.22){
                back_arm_servo_position=0.22;
            }
        }
        //放 手柄2 十字上键
        if(gamepad2.dpad_up){back_arm_servo_position=0.55;}
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
        if(gamepad2.x){claw_servo_position = 0.56;}
        if(gamepad2.b){claw_servo_position = 0.9;}

        robot.back_arm_servo.setPosition(back_arm_servo_position);
        robot.claw_servo.setPosition(claw_servo_position);
    }


    //gamepad1.a  准备吸入标本
    public class down_prepare_xi extends Thread{
        public void run(){
            try{

                forward_arm_servo_position = 0.54;  //舵机中间位置
                //sleep(300);
                //sliderTargetPosition = 0;
                //back_arm_servo_position = 0.22;

                retract_right_servo_position = 0.5;
                retract_left_servo_position = 0.5;
                sleep(300);

            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //gamepad1.y  准备投放标本（到车载篮子里）
    public class down_prepare_tu extends Thread{
        public void run(){
            try{

                forward_arm_servo_position = 0.54;

                sleep(100);
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                sleep(300);
                collect_power  =  0;
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //gamepad2.left_bumper  准备投放标本（到高框里）
    public class up_prepare_fang_gaoKuang extends Thread{
        public void run(){
            try {
                back_arm_servo_position = 0.22;
                forward_arm_servo_position = 0.24; // 收集装置抬到准备吐高度
                retract_left_servo_position = 0.71;
                retract_right_servo_position = 0.29;
                sliderTargetPosition = 0;
                sleep(600);

                collect_using = false;
                robot.collect_motor.setPower(-0.7); //往外吐砖块
                sleep(1000);
                collect_using = true;

                back_arm_servo_position = 0.35;
                sliderTargetPosition = 1900;
                //sleep(1500);

            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    //gamepad2.left_trigger 滑轨降到底（后夹子会张开）
    public class up_prepare_gaoGan_down extends Thread{
        public void run(){
            try {
                back_arm_servo_position=0.22;
                sliderTargetPosition = 0;
                claw_servo_position = 0.62;
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

        }
    }
    //gamepad2.right_bumper  准备收集带有夹子标本（人玩区）
    public class up_prepare_shou_jia extends Thread{
        public void run(){
            try{
                claw_servo_position = 0.34;
                sleep(500);
                sliderTargetPosition = 1400;
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }
    //gamepad2.right_trigger  准备投放带有夹子的标本（到高杆上）
    public class up_prepare_fang_gaoGan extends Thread{
        public void run(){
            try{
                sliderTargetPosition = 1000;
                sleep(300);
                claw_servo_position = 0.62;
                sleep(300);
                sliderTargetPosition = 0;
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }

}
