//package com.example.meepmeeptesting;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class MeepMeepAuto {
//
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(800);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.3)
//                .build();
//        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-37.77, 0.79, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(19.43, 0.79))
//                .splineTo(new Vector2d(18.42, 44.56), Math.toRadians(91.32))
//                .build();
//
//                //shoot 3
//
//
//
//
//
//
//
//
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//}
//
