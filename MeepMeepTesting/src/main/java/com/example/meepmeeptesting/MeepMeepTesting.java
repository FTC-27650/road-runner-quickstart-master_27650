package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.IOException;
import java.util.Objects;

import javax.imageio.ImageIO;

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

        Image background;
        try {
            background = ImageIO.read(
                    Objects.requireNonNull(MeepMeep.class.getResourceAsStream("/background/season-2025-decode/field-2025-juice-dark.png"))
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        meepMeep.setBackground(background)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
