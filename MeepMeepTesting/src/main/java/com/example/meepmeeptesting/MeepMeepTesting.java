package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
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
/*
        //IMPORTANT LOCATIONS
         Pose2d blueCloseStartPose = new Pose2d(14,61,Math.toRadians(270));
         Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(270));
         Pose2d redCloseStartPose = new Pose2d(14,-61,Math.toRadians(90));
         Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(90));

        Pose2d blueCloseMarkerRightPose = new Pose2d(12, 36, Math.toRadians(180));
        Pose2d blueCloseMarkerMiddlePose = new Pose2d(12, 36, Math.toRadians(270));
        Pose2d blueCloseMarkerLeftPose = new Pose2d(12, 36, Math.toRadians(0));

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
*/
        int pixelPlacement = 0;



        Vector2d leftSpikeEndWaypoint = new Vector2d(19.5,47);
        Vector2d middleSpikeEndWaypoint = new Vector2d(11,38.5);
        Vector2d rightSpikeEndWaypoint = new Vector2d(12.5, 41);

        Vector2d backdropStagingWaypoint = new Vector2d(30,54);


        Vector2d backdropLeftEndWaypoint = new Vector2d(51, 43);
        Vector2d backdropMiddleEndWaypoint = new Vector2d(52,37);
        Vector2d backdropRightEndWaypoint = new Vector2d(52,29);

        Vector2d parkStagingWaypoint = new Vector2d(42,60);
        Vector2d parkEndWaypoint = new Vector2d(54,60);


        //Starting position and heading of the robot. *YOU SHOULD NOT NEED TO CHANGE HEADING*
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(42, 30, 3, Math.toRadians(60), 16.05)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                //Drive to Pixel
                                .lineTo(leftSpikeEndWaypoint)
                                //could be wrong if marker offsets chain off eachother
//                                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
//                                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->slideStop())
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, ()->rslideServo.setPosition(0.31))
                                //Leave pixel behind and begin route
                                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(90))

                                //spline to back drop
                                .splineToSplineHeading(new Pose2d(backdropLeftEndWaypoint, Math.toRadians(180)), Math.toRadians(0.1))
                                //Drop pixel
//                                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
//                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> slideRaise())
//                                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> slideStop())
                                .waitSeconds(1)
                                //.UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                                //.UNSTABLE_addTemporalMarkerOffset(1.5,()->slideStop())
                                //back off and prepare for park
                                .lineTo(parkStagingWaypoint)
                                //lower slides
//                                .UNSTABLE_addTemporalMarkerOffset(0,()->rslideServo.setPosition(0.02))
//                                .UNSTABLE_addTemporalMarkerOffset(0.5,()->slideDrop())
                                //park
                                .lineTo(parkEndWaypoint)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}