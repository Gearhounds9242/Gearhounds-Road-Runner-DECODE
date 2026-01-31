package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueCloseGate {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(130), 15.5)
                .build();
        myBot.setDimensions(18, 18);
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -50, Math.toRadians(146.25)))
                .splineToConstantHeading(new Vector2d(-49, -49), Math.toRadians(58.5))
                .splineToSplineHeading(new Pose2d(-20, -20, Math.toRadians(223.5)), Math.toRadians(60))
                .waitSeconds(1)
                //shoot preload3
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(800, 800),
//                                intake.runIntake(1, 0.1),
//                                new SequentialAction(
//                                        new SleepAction(2),
//                                        shooter.shootBallRapid(3, 1, 4)
//                                )
//                        )
//                )
                ///go to 1st spike mark
                .splineToSplineHeading(new Pose2d(-11.5, -29.5, Math.toRadians(270)), Math.toRadians(-40))
                //turn on intake
//                .stopAndAdd(intake.runIntake(1, 0.1))
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(-11.5, -53.5), Math.toRadians(-90))
                //tap transfer
//                .stopAndAdd(transfer.tapTransfer())
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(-14, -40), Math.toRadians(-90))
                //intake off
//                .stopAndAdd(intake.stopIntake())
                /// hit gate
                .splineToSplineHeading(new Pose2d(-9, -30, Math.toRadians(90)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(0, -50), Math.toRadians(-90))
                ///go to shoot location second time
                .splineToLinearHeading(new Pose2d(-20, -20, Math.toRadians(223.5)), Math.toRadians(270))
                //shoot 3
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(800, 800),
//                                intake.runIntake(1, 0.1),
//                                new SequentialAction(
//                                        new SleepAction(2),
//                                        shooter.shootBallRapid(3, 1, 4)
//                                )
//                        )
//                )
                .waitSeconds(1)
                ///go to 2nd spike mark
                .splineToSplineHeading(new Pose2d(12, -29.5, Math.toRadians(270)), Math.toRadians(10))
                //turn on intake
                //.stopAndAdd(intake.runIntake(1,0.1))
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(12, -53.5), Math.toRadians(-90))
                //tap transfer
//                .stopAndAdd(transfer.tapTransfer())
                .waitSeconds(0.05)
                .splineToConstantHeading(new Vector2d(12, -49), Math.toRadians(-90))
                //intake off
//                .stopAndAdd(intake.stopIntake())
                ///go to shoot location third time
                .splineToConstantHeading(new Vector2d(12,-40), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-20, -15, Math.toRadians(223.5)), Math.toRadians(10))
                .waitSeconds(0.05)
                //shoot 3
//                .stopAndAdd(
//                        new ParallelAction(
//                                shooter.runShooter(800, 800),
//                                intake.runIntake(1, 0.1),
//                                new SequentialAction(
//                                        new SleepAction(2),
//                                        shooter.shootBallRapid(3, 1, 4)
//                                )
//                        )
//                )
                ///go out of the launch zone
                .splineToSplineHeading(new Pose2d(-23.5, -50, Math.toRadians(270)), Math.toRadians(270))
                /// save pos for teleop
//                .stopAndAdd(new SavePose())
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}