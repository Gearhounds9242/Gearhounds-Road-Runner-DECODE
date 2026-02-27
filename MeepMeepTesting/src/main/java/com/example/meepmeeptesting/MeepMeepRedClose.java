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

public class MeepMeepRedClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
//        myBot.setDimensions()
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(-146.25)))
                .splineToConstantHeading(new Vector2d(-49, -49), Math.toRadians(58.5))
                        .strafeToLinearHeading(new Vector2d(-19.5,-10), Math.toRadians(223.5))

//                .waitSeconds(0.5)

                        ///shoot preload3
//                        .stopAndAdd(
//                                new ParallelAction(
//                                        shooter.runShooter(topVelocity,bottomVelocity),
//                                        intake.runIntake(1,1),
//                                        new SequentialAction(
//                                                new SleepAction(1),
//                                                transfer.runTransfer(),
//                                                new SleepAction(1),
//                                                transfer.stopTransfer(),
//                                                shooter.stopShooter()
//                                        )
//                                )
//                        )
                        ///go to 1st spike mark
                        .strafeToSplineHeading(new Vector2d(-11.5,-29.5), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-11.5, -53.5), Math.toRadians(-90))
//                //tap transfer
//                        .stopAndAdd(transfer.tapTransfer())
                        .strafeToLinearHeading(new Vector2d(-19.5,-10), Math.toRadians(220))

//                .waitSeconds(0.5)

                        ///shoot 3
//                        .stopAndAdd(
//                                new ParallelAction(
//                                        shooter.runShooter(topVelocity,bottomVelocity),
//                                        intake.runIntake(1,1),
//                                        new SequentialAction(
//                                                new SleepAction(1),
//                                                transfer.runTransfer(),
//                                                new SleepAction(1),
//                                                transfer.stopTransfer(),
//                                                shooter.stopShooter()
//                                        )
//                                )
//                        )


                        .strafeToSplineHeading(new Vector2d(13.5, -22), Math.toRadians(280))
                        .strafeToConstantHeading(new Vector2d(13.5,-60))
                        .strafeToConstantHeading(new Vector2d(13.5,-29))
                        .strafeToLinearHeading(new Vector2d(-19.5,-10), Math.toRadians(220))

//                .waitSeconds(0.5)

//                shoot 3
//                        .stopAndAdd(
//                                new ParallelAction(
//                                        shooter.runShooter(topVelocity,bottomVelocity+100),
//                                        intake.runIntake(1,1),
//                                        new SequentialAction(
//                                                new SleepAction(1),
//                                                transfer.runTransfer(),
//                                                new SleepAction(1),
//                                                transfer.stopTransfer(),
//                                                shooter.stopShooter()
//                                        )
//                                )
//                        )


                        .splineToSplineHeading(new Pose2d(-23.5, -50, Math.toRadians(270)), Math.toRadians(270))

//                /// save pos for teleop
//                        .stopAndAdd(shooter.stopShooter())
//                        .stopAndAdd(new SavePose())

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}