package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -63, 90))
                        .forward(24.5)
                        .turn(Math.toRadians(-20))
                        .forward(2)
                        .splineTo(new Vector2d(-60, -52), Math.toRadians(240))
                        .back(3)
                        .splineToConstantHeading(new Vector2d(-56,-39), Math.toRadians(-90))
                        .turn(Math.toRadians(-140))
//                        .forward(22)
//                        .turn(Math.toRadians(50))
//                        .forward(6.3)
//                        .turn(Math.toRadians(-90))
//                        .back(24)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    };
}