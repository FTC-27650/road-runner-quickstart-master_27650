package org.firstinspires.ftc.teamcode.FTC_27650_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "精简编码器读取")
@Disabled
public class AbsoluteEncoderReader extends LinearOpMode {
    private DcMotorEx encoderMotor;  // 接A/B相
    private DigitalChannel indexPin; // 接I相

    // 关键参数：根据编码器规格修改
    private static final int COUNTS_PER_REV = 8192; // 本例以8192计数/圈为例，如果是2048请修改

    private int zeroOffset = 0; // 归零偏移量
    private boolean lastIndexState = false; // 上次索引状态

    @Override
    public void runOpMode() {
        // 1. 硬件初始化（确保配置中名称为"wheel_encoder"和"index_pin"）
        encoderMotor = hardwareMap.get(DcMotorEx.class, "em");
        indexPin = hardwareMap.get(DigitalChannel.class, "d2");

        // 2. 设置编码器模式
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexPin.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {
            // 3. 读取数据
            int rawCount = encoderMotor.getCurrentPosition();
            boolean indexState = indexPin.getState(); // true=高电平, false=低电平

            // 4. 索引下降沿触发归零（假设索引低电平有效）
            if (!indexState && lastIndexState) {
                zeroOffset = -rawCount;  // 计算偏移量
            }
            lastIndexState = indexState;

            // 5. 计算绝对角度（0-360度）
            int absPos = rawCount + zeroOffset;
            double angleDeg = (absPos % COUNTS_PER_REV) * 360.0 / COUNTS_PER_REV;
            if (angleDeg < 0) angleDeg += 360;

            // 6. 显示必要信息
            telemetry.addData("角度", "%.1f°", angleDeg);
            telemetry.addData("位置", absPos);
            telemetry.addData("索引", indexState ? "高" : "低(零点)");
            telemetry.addData("last索引", lastIndexState ? "高" : "低(零点)");
            telemetry.update();
        }
    }
}