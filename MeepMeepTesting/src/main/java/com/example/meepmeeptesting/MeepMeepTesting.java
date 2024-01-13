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

        //IMPORTANT LOCATIONS
         Pose2d blueCloseStartPose = new Pose2d(14,61,Math.toRadians(90));
         Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(90));
         Pose2d redCloseStartPose = new Pose2d(14,-61,Math.toRadians(270));
         Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(270));

        Pose2d blueCloseMarkerRightPose = new Pose2d(12, 36, Math.toRadians(0));
        Pose2d blueCloseMarkerMiddlePose = new Pose2d(12, 36, Math.toRadians(90));
        Pose2d blueCloseMarkerLeftPose = new Pose2d(12, 36, Math.toRadians(180));

        Pose2d blueClosePostOuttakePose = new Pose2d(12,58,Math.toRadians(180));
        Pose2d blueFarPostOuttakePose = new Pose2d(-36,58,Math.toRadians(180));
        Pose2d redClosePostOuttakePose = new Pose2d(12,-58,Math.toRadians(180));
        Pose2d redFarPostOuttakePose = new Pose2d(-36,-58,Math.toRadians(180));


         Vector2d blueBackDropLeft = new Vector2d(48,40);
         Vector2d redBackDropLeft = new Vector2d(48, -40);

         Vector2d blueBackDropMiddle = new Vector2d(48,35);
         Vector2d redBackDropMiddle = new Vector2d(48, -35);

         Vector2d blueBackDropRight = new Vector2d(48,30);
         Vector2d redBackDropRight = new Vector2d(48, -30);



         Vector2d blueNearPark = new Vector2d(60,60);
         Vector2d blueAwayPark = new Vector2d(60,12);
         Vector2d redNearPark = new Vector2d(60,-60);
         Vector2d redAwayPark = new Vector2d(60,-12);

         Vector2d blueCloseMarkerTile = new Vector2d(12, 36);
         Vector2d blueFarMarkerTile = new Vector2d(-36, 36);
         Vector2d redCloseMarkerTile = new Vector2d(12, -36);
         Vector2d redFarMarkerTile = new Vector2d(-36, -36);

        Pose2d startPose =  blueCloseStartPose;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42.52, 30, 3.7, Math.toRadians(60), 16.05)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineToSplineHeading(blueCloseMarkerMiddlePose)
                                //Outtake of tile goes in this marker
                                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                                .lineToLinearHeading(blueClosePostOuttakePose)
                                .lineTo(blueBackDropMiddle)
                                .strafeRight(23)
                                .back(12)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}