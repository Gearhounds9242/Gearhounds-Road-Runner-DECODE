package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());

        // ==== 红方近场 (Red Close) ====
        RoadRunnerBotEntity redClose = new DefaultBotBuilder(meepMeep)
                // 设置运动学约束: maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.5)
                .build();

        redClose.runAction(
                redClose.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                        .lineToY(-30)      // 前进
                        .turn(Math.toRadians(-90)) // 向右转
                        .lineToX(36)        // 横向移动
                        .turn(Math.toRadians(90))  // 转回
                        .build()
        );


        // ==== 红方远场 (Red Far) ====
        RoadRunnerBotEntity redFar = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.5)
                .build();

        redFar.runAction(
                redFar.getDrive().actionBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                        .lineToY(-20)
                        .turn(Math.toRadians(-90))
                        .lineToX(48)
                        .turn(Math.toRadians(90))
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}