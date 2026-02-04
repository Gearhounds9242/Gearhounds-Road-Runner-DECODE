package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(-146.25)))
                        .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(-4))
                        .waitSeconds(3)
                //    shoot
                        .splineToSplineHeading(new Pose2d(-12,30, Math.toRadians(90)), Math.toRadians(0))
                        .waitSeconds(1)
                       .strafeTo(new Vector2d(-12, 52.5))
                //         intake ball

                        .strafeTo(new Vector2d(-12, 50))
                .splineToLinearHeading(new Pose2d(-12, 52.5, Math.toRadians(90)), Math.toRadians(-11))

                .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(1))
// shot
                        .waitSeconds(1)
                        .splineToSplineHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(0))
                        .strafeTo(new Vector2d(12, 60))
                // intake
                        .strafeTo(new Vector2d(12, 45))
                .splineToSplineHeading(new Pose2d(-11.6, 11.6, Math.toRadians(139)), Math.toRadians(1))
// shoot
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(12, 20))

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}