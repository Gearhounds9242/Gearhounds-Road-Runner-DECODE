package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();
        myBot.setDimensions(18,18);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -22, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(204))
                        .waitSeconds(2)
                .splineTo(new Vector2d(35, -25), Math.toRadians(-90))
//                        .stopAndAdd(intake.runIntake(1, 0.1))
                .strafeTo(new Vector2d(35, -60))
//                        .stopAndAdd(intake.stopIntake())
                .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(220))
                        .waitSeconds(2)
                .splineTo(new Vector2d(12, -30), Math.toRadians(270))
                .strafeTo(new Vector2d(12, -60))
//                        .stopAndAdd(intake.runIntake(1, 0.1))
//                        .stopAndAdd(intake.stopIntake())
                .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(250))
                .waitSeconds(2)

                .splineToSplineHeading(new Pose2d(60, -52.5, Math.toRadians(-90)), Math.toRadians(0))
//                        .stopAndAdd(intake.runIntake(1, 0.1))

                .strafeTo(new Vector2d(60, -60))
                .strafeTo(new Vector2d(60, -45))
//                        .stopAndAdd(intake.stopIntake())


                .strafeToSplineHeading(new Vector2d(55, -14), Math.toRadians(250))
                        .waitSeconds(2)
                .strafeTo(new Vector2d(35, -14))

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}