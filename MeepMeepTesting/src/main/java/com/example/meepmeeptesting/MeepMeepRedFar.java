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
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(55, 14, Math.toRadians(160.7)), Math.toRadians(0))
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

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}