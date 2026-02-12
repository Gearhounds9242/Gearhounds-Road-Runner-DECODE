package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 70, Math.PI * 1.5, Math.PI * 1.5, 13)

                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(157)), Math.toRadians(0))
                        .waitSeconds(3)
//              add shoot
                .splineTo(new Vector2d(35,30), Math.toRadians(90))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(35, 60))
 //               intake balls
                .strafeTo(new Vector2d(35, 45))
                        .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
                .waitSeconds(3)
//              add shoot
                .splineTo(new Vector2d(12,30), Math.toRadians(90))
                .waitSeconds(1)
                        .strafeTo(new Vector2d(12, 60))
                //               intake balls
                                .strafeTo(new Vector2d(12, 45))
                .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
                        .waitSeconds(3)
                //              add shoot
                        .splineToSplineHeading(new Pose2d(60, 52.5, Math.toRadians(90)), Math.toRadians(0))

                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 45))

                .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}