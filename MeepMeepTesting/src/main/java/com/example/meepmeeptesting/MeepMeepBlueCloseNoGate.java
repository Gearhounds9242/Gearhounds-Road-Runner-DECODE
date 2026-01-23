package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueCloseNoGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
        myBot.setDimensions(18,18);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(146.25)))
                .splineToConstantHeading(new Vector2d(-49,-49), Math.toRadians(58.5))
                .splineToSplineHeading(new Pose2d(-20, -20, Math.toRadians(223.5)), Math.toRadians(60))
                .waitSeconds(1)
//              //shoot preload3
                .splineToSplineHeading(new Pose2d(-11.5,-29.5, Math.toRadians(270)), Math.toRadians(-40))
//              //turn on intake
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(-11.5, -53.5),Math.toRadians(-90))
//              //tap transfer
//              .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(-11.5,-49), Math.toRadians(-90))
                //intake off
                .splineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(223.5)), Math.toRadians(270))
                //shoot 3
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(12,-29.5, Math.toRadians(270)), Math.toRadians(10))
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(12, -53.5),Math.toRadians(-90))
                //turn on intake
                .splineToConstantHeading(new Vector2d(12,-49), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12,-49), Math.toRadians(-90))
                //intake off
                .splineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(223.5)), Math.toRadians(10))
                .waitSeconds(0.05)
                //shoot 3
                .splineToSplineHeading(new Pose2d(-23.5,-50, Math.toRadians(270)), Math.toRadians(270))
                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}