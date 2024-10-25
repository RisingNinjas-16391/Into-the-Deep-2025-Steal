package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, -61.75, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-8, -37.75))
                .waitSeconds(1.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47.1, -46.5, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(1.5)
                .turnTo(Math.toRadians(225))
                .strafeToConstantHeading(new Vector2d(-52.6, -53.1))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-58.8, -46.5, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-52.6, -53.1, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-47.1, -46.5, Math.toRadians(315)), Math.toRadians(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-52.6, -53.1, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-52.6, -53.1, Math.toRadians(225)), Math.toRadians(180))








                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}