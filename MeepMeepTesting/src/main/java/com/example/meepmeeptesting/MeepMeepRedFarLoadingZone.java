package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedFarLoadingZone {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);



        int topVelocity = 1215;
        int bottomVelocity = 1215;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 70, Math.PI * 1.5, Math.PI * 1.5, 13)

                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14, Math.toRadians(180)))

                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                                .waitSeconds(1)
                // shoot
                        .strafeToSplineHeading(new Vector2d(60, 14), Math.toRadians(90))
                        .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
                // intake
                        .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
                // intake
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
                // intake
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
                // intake
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))
                // intake
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(30, 30, Math.toRadians(90)), Math.toRadians(0))



                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}