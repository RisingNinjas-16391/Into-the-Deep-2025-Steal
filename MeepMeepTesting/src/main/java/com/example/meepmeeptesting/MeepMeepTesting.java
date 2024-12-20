package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.25)
                .build();

        myBot1.runAction(myBot1.getDrive().actionBuilder(new Pose2d(-8, -61.75, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(-8, -37.75))
                .waitSeconds(2.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47.1, -46.5, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(2.0)
                .setReversed(false)
                .turnTo(Math.toRadians((225)))
                .waitSeconds(2.0)
                .turnTo(Math.toRadians((270)))
                .strafeToConstantHeading(new Vector2d(-58.8, -46.5))
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-52.1, -46.5))
                .turnTo(Math.toRadians(320))
                .waitSeconds(2.0)
                .turnTo(Math.toRadians(225))
                .waitSeconds(2.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-28.2, -15, Math.toRadians(0)), Math.toRadians(45))
                .waitSeconds(1.0)
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(-26.2, -15))

                .build());

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.25)
                .build();

        myBot3.runAction(myBot3.getDrive().actionBuilder(new Pose2d(8, 61.75, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(8, 37.75))
                .waitSeconds(2.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(47.1, 46.5, Math.toRadians(90)), Math.toRadians(60))
                .waitSeconds(2.0)
                .setReversed(false)
                .turnTo(Math.toRadians(45))
                .waitSeconds(2)
                .turnTo(Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(58.8, 46.5))
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(52.1, 46.5))
                .turnTo(Math.toRadians(140))
                .waitSeconds(2.0)
                .turnTo(Math.toRadians(45))
                .waitSeconds(2.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(28.2, 15, Math.toRadians(180)), Math.toRadians(225))
                .waitSeconds(1.0)
                .setReversed(false)
                .strafeToConstantHeading(new Vector2d(26.2, 15))

                .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.25)

                .build();

        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(8, -61.75, Math.toRadians(90)))

                .strafeToConstantHeading(new Vector2d(8, -37.75))
                .waitSeconds(1.0)
                .strafeToConstantHeading(new Vector2d(8, -39.75))
                .splineToLinearHeading(new Pose2d(34.5, -10.1, Math.toRadians(270)), Math.toRadians(93))
                .strafeToConstantHeading(new Vector2d(46, -10.1))
                .strafeToConstantHeading(new Vector2d(46, -50.1))
                .strafeToConstantHeading(new Vector2d(46, -10.1))
                .strafeToConstantHeading(new Vector2d(53, -10.1))
                .strafeToConstantHeading(new Vector2d(53, -50.1))
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(5, -39.75, Math.toRadians(90)), Math.toRadians(93))
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(61, -10.1, Math.toRadians(270)), Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(61, -54.5))
                .strafeToConstantHeading(new Vector2d(53, -50.1))
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(3, -39.75, Math.toRadians(90)), Math.toRadians(93))
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(53, -50.1, Math.toRadians(270)), Math.toRadians(267))
                .waitSeconds(1.0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(6.5, -39.75, Math.toRadians(90)), Math.toRadians(93))

                .build());


        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.25)
                .build();

        myBot4.runAction(myBot4.getDrive().actionBuilder(new Pose2d(-8, 61.75, Math.toRadians(270)))

                .strafeToConstantHeading(new Vector2d(-8, 37.75))
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-8, 39.75))
                .splineToConstantHeading(new Vector2d(-34.5, 10.1), Math.toRadians(273))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 54.5))
                .build());





        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot1)
                .addEntity(myBot2)
//                .addEntity(myBot3)
                .addEntity(myBot4)
                .start();
    }
}