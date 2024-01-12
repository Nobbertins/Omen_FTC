package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PlannedTrajectories {
    private SampleMecanumDrive drive;
    public PlannedTrajectories(HardwareMap hardwareMap){

        this.drive = new SampleMecanumDrive(hardwareMap);
    }

    //Fill this function with the functionality to Outtake the pixel on the line and any other actual robotics shit, runs during Outtake sequence
    public void OuttakePlaceHolder(){

    }

    //Fill this function with the functionality to drop off a pixel on the backdrop
    public void dropPlaceHolder(){

    }



    //IMPORTANT LOCATIONS
    Pose2d blueCloseStartPose = new Pose2d(12,61,Math.toRadians(270));
    Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(270));
    Pose2d redCloseStartPose = new Pose2d(12,-61,Math.toRadians(90));
    Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(90));

    Pose2d blueClosePostOuttakePose = new Pose2d(12,58,Math.toRadians(0));
    Pose2d blueFarPostOuttakePose = new Pose2d(-36,58,Math.toRadians(0));
    Pose2d redClosePostOuttakePose = new Pose2d(12,-58,Math.toRadians(0));
    Pose2d redFarPostOuttakePose = new Pose2d(-36,-58,Math.toRadians(0));


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

    //neccesary to prevent mismatch between the driver's representation of pose and the trajectory's
    //only need to do this once for the start pose in the beginning if you chain trajectory sequences

    //All trajectories for automatic Outtake of pixel on marker's line

    //Close Blue
    TrajectorySequence blueCloseMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            //TESTING A SPLINE HEADING

            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    TrajectorySequence blueCloseMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            //TESTING A SPLINE HEADING

            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    TrajectorySequence blueCloseMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading

            //TESTING A SPLINE HEADING
            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    //Far Blue
    TrajectorySequence blueFarMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    TrajectorySequence blueFarMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    TrajectorySequence blueFarMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    //Close red
    TrajectorySequence redCloseMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    TrajectorySequence redCloseMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    TrajectorySequence redCloseMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    //Far red
    TrajectorySequence redFarMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redFarPostOuttakePose)
            .build();

    TrajectorySequence redFarMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redFarPostOuttakePose)
            .build();

    TrajectorySequence redFarMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redFarPostOuttakePose)
            .build();


        /*
        Drop yellow pixel in intake sequence
        because all Outtakes return to the same spot the same drop sequence can be utilized for all sequences of same start pose
         */

    //Close Blue
    TrajectorySequence blueCloseDropMiddleSequence = drive.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    TrajectorySequence blueCloseDropRightSequence = drive.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    TrajectorySequence blueCloseDropLeftSequence = drive.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropLeft, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();


    //Far Blue
    TrajectorySequence blueFarDropMiddleSequence = drive.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropMiddle, Math.toRadians(0))
            .build();
    TrajectorySequence blueFarDropRightSequence = drive.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropRight, Math.toRadians(0))
            .build();
    TrajectorySequence blueFarDropLeftSequence = drive.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropLeft, Math.toRadians(0))
            .build();


    //Close red
    TrajectorySequence redCloseDropMiddleSequence = drive.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    TrajectorySequence redCloseDropRightSequence = drive.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    TrajectorySequence redCloseDropLeftSequence = drive.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Far red
    TrajectorySequence redFarDropMiddleSequence = drive.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    TrajectorySequence redFarDropRightSequence = drive.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    TrajectorySequence redFarDropLeftSequence = drive.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropLeft, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Trajectories for parking after dropping off pixel, they are the same for all blue starts and all red starts

    //Blue Near
    TrajectorySequence blueNearParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDropMiddle, Math.toRadians(0)))
            .strafeLeft(23)
            .forward(12)
            .build();
    //Blue Away
    TrajectorySequence blueAwayParkSequence = drive.trajectorySequenceBuilder(new Pose2d(blueBackDropMiddle, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeRight(24)
            .forward(12)
            .build();
    //Red Near
    TrajectorySequence redNearParkSequence = drive.trajectorySequenceBuilder(new Pose2d(redBackDropMiddle, Math.toRadians(0)))
            .strafeRight(23)
            .forward(12)
            .build();
    //Red Away
    TrajectorySequence redAwayParkSequence = drive.trajectorySequenceBuilder(new Pose2d(redBackDropMiddle, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeLeft(24)
            .forward(12)
            .build();




}
