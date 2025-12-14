package org.firstinspires.ftc.teamcode.FTC_27650_Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class test extends LinearOpMode {


    public void runOpMode() {


        waitForStart();

        while (opModeIsActive()) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Pose2d currentPose = drive.localizer.getPose();

            // 提取坐标和角度
            double x = currentPose.position.x;        // X坐标（英寸）
            double y = currentPose.position.y;        // Y坐标（英寸）
            double heading = currentPose.heading.toDouble(); // 方位角（弧度）

            // 转换为角度
            double headingDegrees = Math.toDegrees(heading);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading (deg)", headingDegrees);
            telemetry.update();

            Pose2d pose = drive.localizer.getPose();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }
    }
}
