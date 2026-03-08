package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-54.04, 43.94, Math.toRadians(130)))
                .lineToY(27)
                .waitSeconds(1.5)
                .setTangent(-45)
                .splineToSplineHeading(new Pose2d(11.5, 23.6, Math.toRadians(90)), Math.toRadians(90),new TranslationalVelConstraint(80))
                .splineToSplineHeading(new Pose2d(11.5, 60, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(180)
                .splineToLinearHeading(new Pose2d(5, 52, Math.toRadians(0)),Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}