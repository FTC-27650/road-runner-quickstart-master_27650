package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    @SuppressWarnings("FeatureEnvy")
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // 设置机器人约束：最大速度、最大加速度、最大角速度、最大角加速度、轨迹宽度
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-62.20, 37.8, Math.toRadians(270)))
                        .lineToLinearHeading(new Pose2d(12, 10, Math.toRadians(190)))
                        .lineToSplineHeading(new Pose2d(50, 50, Math.toRadians(0)))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
