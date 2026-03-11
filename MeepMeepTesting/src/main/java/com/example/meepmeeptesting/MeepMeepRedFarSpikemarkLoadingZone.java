package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedFarSpikemarkLoadingZone {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16.5,18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(85, 70, Math.PI * 1.5, Math.PI * 1.5, 13)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, 14, Math.toRadians(180)))

                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                // shoot

                //pick up
                .strafeToSplineHeading(new Vector2d(35,25), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(35, 60))
//                .stopAndAdd(transfer.tapTransfer())

                //shoot
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(152))
                .waitSeconds(1)

                // intake
                .strafeToLinearHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 50))
                .strafeTo(new Vector2d(60, 60))

                //Shoot
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)


                // intake
                .strafeToSplineHeading(new Vector2d(35,25), Math.toRadians(90))
                .strafeTo(new Vector2d(35, 60))
                .strafeTo(new Vector2d(35, 50))
                .strafeTo(new Vector2d(35, 60))

                //Shoot
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)


                // intake
                .strafeToLinearHeading(new Vector2d(60, 14), Math.toRadians(90))
                .strafeTo(new Vector2d(60, 60))
                .strafeTo(new Vector2d(60, 50))
                .strafeTo(new Vector2d(60, 60))


                //Shoot
                .strafeToSplineHeading(new Vector2d(55, 14), Math.toRadians(156))
                .waitSeconds(1)

                .strafeToSplineHeading(new Vector2d(60, 60), Math.toRadians(90))

                .build());








        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}