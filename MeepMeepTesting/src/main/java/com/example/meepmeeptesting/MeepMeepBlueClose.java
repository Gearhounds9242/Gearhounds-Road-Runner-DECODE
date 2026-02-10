package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueClose {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 70, Math.PI * 1.5, Math.PI * 1.5, 13)

                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(146.25)))
                ///go to shoot location first time
                .splineToConstantHeading(new Vector2d(-49, -49), Math.toRadians(58.5))
                .strafeToLinearHeading(new Vector2d(-11,-11), Math.toRadians(223.5))

                .waitSeconds(2)

                ///shoot preload3
//                .stopAndAdd(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        intake.runIntake(1,1),
//                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
//                                ),
//                                transfer.runTransfer(),
//                                new SleepAction(1),
//                                transfer.stopTransfer(),
//                                shooter.stopShooter()
//                        )
//                )
                ///go to 1st spike mark
                .strafeToSplineHeading(new Vector2d(-11.5,-29.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-11.5, -53.), Math.toRadians(-90))
//                //tap transfer
///                .stopAndAdd(transfer.tapTransfer())
                .strafeToSplineHeading(new Vector2d(-10,-10), Math.toRadians(223.5))

                .waitSeconds(2)

                ///shoot 3
//                .stopAndAdd(
//                        new SequentialAction(
//                                new ParallelAction(
//                                        intake.runIntake(1,1),
//                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
//                                ),
//                                transfer.runTransfer(),
//                                new SleepAction(1),
//                                transfer.stopTransfer(),
//                                shooter.stopShooter()
//                        )
//                )


                .strafeToSplineHeading(new Vector2d(11.5, -29), Math.toRadians(275))
                .strafeToConstantHeading(new Vector2d(11.5,-60))
                .strafeToConstantHeading(new Vector2d(11.5,-29))
                .strafeToSplineHeading(new Vector2d(-10,-10), Math.toRadians(223.5))

                .waitSeconds(2)

//                //shoot 3
////                .stopAndAdd(
////                        new SequentialAction(
////                                new ParallelAction(
////                                        intake.runIntake(1,1),
////                                        shooter.shootSequence(topVelocity, bottomVelocity, 3, 1, 1)
////                                ),
////                                transfer.runTransfer(),
////                                new SleepAction(1),
////                                transfer.stopTransfer(),
////                                shooter.stopShooter()
////                        )
////                )


                .splineToSplineHeading(new Pose2d(-23.5, -50, Math.toRadians(270)), Math.toRadians(270))

//                /// save pos for teleop
////                .stopAndAdd(new SavePose())
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}