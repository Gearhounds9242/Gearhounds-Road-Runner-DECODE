package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 70, Math.PI * 1.5, Math.PI * 1.5, 13)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(150))
//                .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,-10))
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(topVelocity, bottomVelocity),
//                                intake.runIntake(1, 1),
//                                new SequentialAction(
//                                        new SleepAction(1.5),
//                                        transfer.runTransfer(),
//                                        new SleepAction(1),
//                                        transfer.stopTransfer(),
//                                        shooter.stopShooter()
//                                )
//                        )
//                )
                .strafeToSplineHeading(new Vector2d(35, 25), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(35, 60))
//                .stopAndAdd(transfer.tapTransfer())
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(155))
//                .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,-2))
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(topVelocity, bottomVelocity),
//                                new SequentialAction(
//                                        new SleepAction(1),
//                                        transfer.runTransfer(),
//                                        new SleepAction(1),
//                                        transfer.stopTransfer(),
//                                        shooter.stopShooter()
//                                )
//                        )
//                )
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(60, 20, Math.toRadians(90)), Math.toRadians(0))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 50))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 50))
                .strafeTo(new Vector2d(60, 60))
//                .stopAndAdd(transfer.tapTransfer())
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(158))
//                .stopAndAdd(vision.alignToTag(Vision.RED_GOAL_TAG_ID,2.1))
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(topVelocity-10, bottomVelocity-10),
//                                intake.runIntake(1, 1),
//                                new SequentialAction(
//                                        new SleepAction(1),
//                                        transfer.runTransfer(),
//                                        new SleepAction(1),
//                                        transfer.stopTransfer(),
//                                        shooter.stopShooter(),
//                                        intake.stopIntake()
//                                )
//                        )
//                )
                .strafeToLinearHeading(new Vector2d(50, 50), Math.toRadians(90))


                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}