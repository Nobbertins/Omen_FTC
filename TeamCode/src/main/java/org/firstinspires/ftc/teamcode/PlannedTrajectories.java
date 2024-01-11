package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PlannedTrajectories {
    private SampleMecanumDrive drive;
    public PlannedTrajectories(SampleMecanumDrive drive){
        this.drive = drive;
    }

    //Fill this function with the functionality to outake the pixel on the line and any other actual robotics shit, runs during outake sequence
    public void outakePlaceHolder(){

    }

    //Fill this function with the functionality to drop off a pixel on the backdrop
    public void dropPlaceHolder(){

    }



    //IMPORTANT LOCATIONS
    Pose2d blueCloseStartPose = new Pose2d(12,61,Math.toRadians(270));
    Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(270));
    Pose2d redCloseStartPose = new Pose2d(12,-61,Math.toRadians(90));
    Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(90));

    Pose2d blueClosePostOutakePose = new Pose2d(12,58,Math.toRadians(0));
    Pose2d blueFarPostOutakePose = new Pose2d(-36,58,Math.toRadians(0));
    Pose2d redClosePostOutakePose = new Pose2d(12,-58,Math.toRadians(0));
    Pose2d redFarPostOutakePose = new Pose2d(-36,-58,Math.toRadians(0));

    Vector2d blueBackDrop = new Vector2d(48,36);
    Vector2d redBackDrop = new Vector2d(48, -36);

    Vector2d blueNearPark = new Vector2d(60,60);
    Vector2d blueAwayPark = new Vector2d(60,12);
    Vector2d redNearPark = new Vector2d(60,-60);
    Vector2d redAwayPark = new Vector2d(60,-12);

    Vector2d blueCloseMarkerTile = new Vector2d(12, 36);
    Vector2d blueFarMarkerTile = new Vector2d(-36, 36);
    Vector2d redCloseMarkerTile = new Vector2d(12, -36);
    Vector2d redFarMarkerTile = new Vector2d(-36, -36);

    //neccesary to prevent mismatch between the driver's representation of pose and the trajectory's
    //only need to do this once for the start pose in the beginning if you chain trajectory sequences

    //All trajectories for automatic outake of pixel on marker's line

    //Close Blue
    TrajectorySequence blueCloseMarkerRightOutakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(180)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueClosePostOutakePose)
            .build();

    TrajectorySequence blueCloseMarkerMiddleOutakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOutakePose)
            .build();

    TrajectorySequence blueCloseMarkerLeftOutakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(0)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOutakePose)
            .build();

    //Far Blue
    TrajectorySequence blueFarMarkerRightOutakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(180)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueFarPostOutakePose)
            .build();

    TrajectorySequence blueFarMarkerMiddleOutakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(270)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOutakePose)
            .build();

    TrajectorySequence blueFarMarkerLeftOutakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(0)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOutakePose)
            .build();

    //Close red
    TrajectorySequence redCloseMarkerRightOutakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redClosePostOutakePose)
            .build();

    TrajectorySequence redCloseMarkerMiddleOutakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(270)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOutakePose)
            .build();

    TrajectorySequence redCloseMarkerLeftOutakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(0)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOutakePose)
            .build();

    //Far red
    TrajectorySequence redFarMarkerRightOutakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(180)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redFarPostOutakePose)
            .build();

    TrajectorySequence redFarMarkerMiddleOutakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(270)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redFarPostOutakePose)
            .build();

    TrajectorySequence redFarMarkerLeftOutakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(0)))
            //Outake of tile goes in this marker
            .addDisplacementMarker(() ->{outakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redFarPostOutakePose)
            .build();


        /*
        Drop yellow pixel in intake sequence
        because all outakes return to the same spot the same drop sequence can be utilized for all sequences of same start pose
         */

    //Close Blue
    TrajectorySequence blueCloseDropSequence = drive.trajectorySequenceBuilder(blueClosePostOutakePose)
            .splineTo(blueBackDrop, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Far Blue
    TrajectorySequence blueFarDropSequence = drive.trajectorySequenceBuilder(blueFarPostOutakePose)
            .lineToLinearHeading(blueClosePostOutakePose)
            .splineTo(blueBackDrop, Math.toRadians(0))
            .build();

    //Close red
    TrajectorySequence redCloseDropSequence = drive.trajectorySequenceBuilder(redClosePostOutakePose)
            .splineTo(redBackDrop, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Far red
    TrajectorySequence redFarDropSequence = drive.trajectorySequenceBuilder(redFarPostOutakePose)
            .lineToLinearHeading(redClosePostOutakePose)
            .splineTo(redBackDrop, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Trajectories for parking after dropping off pixel, they are the same for all blue starts and all red starts

    //Blue Near
    TrajectorySequence blueNearParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDrop, Math.toRadians(0)))
            .strafeLeft(23)
            .forward(12)
            .build();
    //Blue Away
    TrajectorySequence blueAwayParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDrop, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeRight(24)
            .forward(12)
            .build();
    //Red Near
    TrajectorySequence redNearParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDrop, Math.toRadians(0)))
            .strafeRight(23)
            .forward(12)
            .build();
    //Red Away
    TrajectorySequence redAwayParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDrop, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeLeft(24)
            .forward(12)
            .build();




}
