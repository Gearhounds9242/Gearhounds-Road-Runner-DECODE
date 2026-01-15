package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(146.25)))
                        .splineToConstantHeading(new Vector2d(-45,-45),Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(-16, -16, Math.toRadians(223.5)), Math.toRadians(25.5))
                                .waitSeconds(1)
                //shoot preload3
                        .splineToSplineHeading(new Pose2d(-11.5,-26, Math.toRadians(270)), Math.toRadians(-40))
                                .waitSeconds(1)
                //turn on intake
                        .splineToConstantHeading(new Vector2d(-11.5, -53.5),Math.toRadians(-90))
                                .waitSeconds(1)
                //tap transfer
                .splineToConstantHeading(new Vector2d(-11.5, -45),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-16, -16, Math.toRadians(223.5)), Math.toRadians(-40))
                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}