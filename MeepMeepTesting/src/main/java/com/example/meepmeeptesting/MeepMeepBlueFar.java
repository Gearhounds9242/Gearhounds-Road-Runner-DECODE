package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueFar {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15.5)
                .build();
        myBot.setDimensions(18,18);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -22, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(56,-22, Math.toRadians(195)), Math.toRadians(200))
                        .waitSeconds(5)
                .splineToConstantHeading(new Vector2d(40,-20),Math.toRadians(180))
                       .splineToSplineHeading(new Pose2d(34.8,-30, Math.toRadians(270)), Math.toRadians(250))
                .splineToConstantHeading(new Vector2d(34.8,-51),Math.toRadians(258))
                        .splineToConstantHeading(new Vector2d(40,-20),Math.toRadians(180))
               //       .splineToSplineHeading(new Pose2d(60,-22, Math.toRadians(195)), Math.toRadians(250))
                    .splineToSplineHeading(new Pose2d(56,-22, Math.toRadians(130)), Math.toRadians(150))
                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}