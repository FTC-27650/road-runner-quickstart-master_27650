package org.firstinspires.ftc.teamcode.ftc_test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "快速LED控制")
@Disabled
public class FastLedControl extends LinearOpMode {
    private DigitalChannel signalPin;
    private long lastPressTime = 0;

    @Override
    public void runOpMode() {
        signalPin = hardwareMap.get(DigitalChannel.class, "d2");
        signalPin.setMode(DigitalChannel.Mode.OUTPUT);
        signalPin.setState(true); // 默认高电平

        telemetry.addData("控制", "A:红(短脉冲) | B:绿(中脉冲) | X:白(长脉冲) | Y:关(超长脉冲)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 防连击：最小按键间隔80ms
            if (System.currentTimeMillis() - lastPressTime > 80) {
                if (gamepad1.a) {
                    sendShortPulse(20); // 20ms短脉冲 = 红
                    lastPressTime = System.currentTimeMillis();
                    telemetry.addData("状态", "红色");
                } else if (gamepad1.b) {
                    sendShortPulse(45); // 45ms中脉冲 = 绿
                    lastPressTime = System.currentTimeMillis();
                    telemetry.addData("状态", "绿色");
                } else if (gamepad1.x) {
                    sendShortPulse(70); // 70ms长脉冲 = 白
                    lastPressTime = System.currentTimeMillis();
                    telemetry.addData("状态", "白色");
                } else if (gamepad1.y) {
                    sendShortPulse(120); // 120ms超长脉冲 = 关
                    lastPressTime = System.currentTimeMillis();
                    telemetry.addData("状态", "关闭");
                }
            }

            telemetry.update();
        }

        // 程序结束发送关灯脉冲
        sendShortPulse(120);
        signalPin.setState(true);
    }

    // 发送单个精确宽度的脉冲
    private void sendShortPulse(int widthMs) {
        signalPin.setState(false);
        sleep(widthMs);
        signalPin.setState(true);
    }
}