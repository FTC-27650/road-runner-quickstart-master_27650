package org.firstinspires.ftc.teamcode.FTC_27650_Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "limelight", group = "limelight")
@Config
public class limelight extends LinearOpMode {
    public static double limelightMountAngleDegrees = 25.0; // 您的limelight从完全垂直方向向后旋转了多少度？
    public static double limelightLensHeightInches = 10; // Limelight镜头中心到地面的距离
    public static double goalHeightInches = 24.0;// 目标到地面的距离

    @Override

    public void runOpMode() throws InterruptedException {
        Limelight3A limelight;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(90); // 这设置了我们向Limelight请求数据的频率（每秒90次）
        limelight.pipelineSwitch(0); // 切换到管道编号0


        waitForStart();
        while (opModeIsActive()) {
            limelight.start(); // 这告诉Limelight开始观察！
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // 目标在左右方向上的偏移（度）
                double ty = result.getTy(); // 目标在上下方向上的偏移（度）
                double ta = result.getTa(); // 目标看起来有多大（图像的0%-100%）

                telemetry.addData("目标 X", tx);
                telemetry.addData("目标 Y", ty);
                telemetry.addData("目标面积", ta);

                // Estimating_Distance（估算距离）方法的使用示例
                // 你需要提供实际的安装角度和高度

                double distance = Estimating_Distance(ty, limelightMountAngleDegrees, limelightLensHeightInches, goalHeightInches);
                telemetry.addData("Estimated Distance", distance);

            } else {
                telemetry.addData("Limelight", "没有目标");
            }
            telemetry.update();
        }
    }

    public double Estimating_Distance(double targetOffsetAngle_Vertical, double limelightMountAngleDegrees, double limelightLensHeightInches, double goalHeightInches) {
        // NetworkTables 代码已被移除，因为它是为 FRC 设计的，不适用于 FTC。
        // 我们直接使用传入的 targetOffsetAngle_Vertical (ty)。

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //计算距离

        // 必须返回计算出的值
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

}
