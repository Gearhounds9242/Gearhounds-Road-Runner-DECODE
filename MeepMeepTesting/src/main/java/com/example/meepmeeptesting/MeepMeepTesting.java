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
//                .strafeTo(new Vector2d(55, 15))
//                .splineToLinearHeading(new Pose2d(50, 15, Math.toRadians(25)), Math.toRadians(0))
//                                .waitSeconds(10)
// servo open/close with delay
//\                .stopAndAdd(new DropUp())
                .strafeTo(new Vector2d(52, 15))
//                .splineToLinearHeading(new Pose2d(50, 15, Math.toRadians(-25)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, 18, Math.toRadians(-22.5)), Math.toRadians(0))
//                .stopAndAdd(new StartShooterStrong())   // spin up
                .waitSeconds(3)
//                .stopAndAdd(new ShootBall())
                .waitSeconds(3)
//                .stopAndAdd(new StartShooter())
                .waitSeconds(3)// spin up
//                .stopAndAdd(new ShootBall())
                .waitSeconds(3)
//                .stopAndAdd(new ShootBall())
                .waitSeconds(3)
//                .stopAndAdd(new StopShooter())


                .splineToLinearHeading(new Pose2d(58, 36, Math.toRadians(90)), Math.toRadians(0))
                .strafeTo(new Vector2d(60, 36))
//                .stopAndAdd(new SavePose())
                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}