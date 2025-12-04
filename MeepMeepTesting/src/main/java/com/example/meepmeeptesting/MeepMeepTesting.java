package com.example.meepmeeptesting;

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
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 17.3)
                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 15, Math.toRadians(0)))
                //shoot 3




//                .splineToSplineHeading(
//                        new Pose2d(30, 20, Math.toRadians(90)),
//                        Math.toRadians(90) // arriving toward +X
//                )


//                .splineToLinearHeading(new Pose2d(-52, 38, Math.toRadians(90)), Math.toRadians(200))



//                .splineToLinearHeading(new Pose2d(50,13, Math.toRadians(0)), Math.toRadians(0))
//                .strafeTo(new Vector2d(50,15))
                .strafeTo(new Vector2d(55,15))
                .splineToLinearHeading(new Pose2d(50,15,Math.toRadians(155)), Math.toRadians(0))
//                .waitSeconds(3)
                .splineToLinearHeading(new Pose2d(55,18,Math.toRadians(155)),Math.toRadians(0))
                .waitSeconds(3)//SHOOT 3 HERE
                .splineToLinearHeading(new Pose2d(58, 36, Math.toRadians(90)), Math.toRadians(0))
                .strafeTo(new Vector2d(60,36))




//                        .strafeTo(new Vector2d(55,36))
//                                .turn(Math.toRadians(90))
//                                .strafeTo(new Vector2d( 58,36))
                .build());






        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}