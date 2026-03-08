package org.firstinspires.ftc.teamcode.FTC_27650_AUTO_Test;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "SplineTest_a", group = "Autonomous")
public final class SplineTest_a extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-54.04, 43.94, Math.toRadians(130));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToY(27)
                        .waitSeconds(1.5)
                        .setTangent(-45)
                        .splineToSplineHeading(new Pose2d(11.5, 23.6, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(80))
                        .splineToSplineHeading(new Pose2d(11.5, 60, Math.toRadians(90)), Math.toRadians(90))
                        .setTangent(180)
                        .splineToLinearHeading(new Pose2d(5, 52, Math.toRadians(0)), Math.toRadians(180))
                        .build());

    }
}
