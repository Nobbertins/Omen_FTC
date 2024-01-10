package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/*
HOW TO MAKE YOUR OWN RUN CONFIGURATION
    1: Go To Edit Configuration Where It Currently Says Team Code
    2: Press the Plus Button In Top Left Corner
    3: Name Your run Configuration meepmeep-run for consistency sake
    4: Make -cp Real_FTC.MeepMeepTesting.main
    5: Make main class .com.example.meepmeeptesting.MeepMeepTesting
    6: Apply and Smile :)
 */


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //EVERY TILE IS 24x24



        Pose2d blueCloseStartPose = new Pose2d(12,56,Math.toRadians(270));
        Pose2d blueFarStartPose = new Pose2d(-35,56,Math.toRadians(270));
        Pose2d redCloseStartPose = new Pose2d(12,-56,Math.toRadians(90));
        Pose2d redFarStartPose = new Pose2d(-35,-56,Math.toRadians(90));

        Pose2d startPose = blueCloseStartPose;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(46, 36), Math.toRadians(0))
                                .waitSeconds(3)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}