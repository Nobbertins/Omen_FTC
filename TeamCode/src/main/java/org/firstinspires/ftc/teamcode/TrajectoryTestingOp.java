package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajectoryTestingOp extends LinearOpMode {

    /*
    STANDARDS FOR TRAJECTORY MAKING:
        -Look at all standards defined by FTC/Road Runner ( (0,0) is center of field, red alliance depo is top left etc.)
        -Every tile is ~24 units
        -Marker Lines are defined by their position (left, right, middle) when viewed from outside the field with the backdrop considered left
        -"Close" refers to a start that is closer to the backdrop (higher X-Value) and "Far" the opposite
        -"Near" refers to a location closer to the driver's wall (higher Y-Value) while "Away" means the opposite
        -Splines are very useful to avoid discontinuity, however avoid using splines anytime you can get away with it because our bot is hit or miss with splines
        -All trajectories are to be of type TrajectorySequence that means no TrajectoryBuilder
        -Use +/- 2 field coordinates as the average error of our bot's movement, this means always give at least 2 units of space from obstacles whenever possible
        -Use marker offsets whenever possible
        -if you are using a non-offset marker place it after the event it will begin during
        - See: https://github.com/NoahBres/MeepMeep for setting up the meepmeep module on your clone for path visualization
     */
    
    //Fill this function with the functionality to outtake the pixel on the line and any other actual robotics shit, runs during outtake sequence
    public void outtakePlaceHolder(){
        
    }

    //Fill this function with the functionality to drop off a pixel on the backdrop
    public void dropPlaceHolder(){

    }
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);



        //IMPORTANT LOCATIONS
        Pose2d blueCloseStartPose = new Pose2d(12,61,Math.toRadians(270));
        Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(270));
        Pose2d redCloseStartPose = new Pose2d(12,-61,Math.toRadians(90));
        Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(90));

        Pose2d blueClosePostouttakePose = new Pose2d(12,58,Math.toRadians(0));
        Pose2d blueFarPostouttakePose = new Pose2d(-36,58,Math.toRadians(0));
        Pose2d redClosePostouttakePose = new Pose2d(12,-58,Math.toRadians(0));
        Pose2d redFarPostouttakePose = new Pose2d(-36,-58,Math.toRadians(0));

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

        Pose2d startPose = blueCloseStartPose;

        //necessary to prevent mismatch between the driver's representation of pose and the trajectory's
        //only need to do this once for the start pose in the beginning if you chain trajectory sequences
        drive.setPoseEstimate(startPose);

        //All trajectories for automatic outtake of pixel on marker's line

        //Close Blue
        TrajectorySequence blueCloseMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(180)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                //giving more lee-way to wall because bot was hitting wall during testing
                .lineToLinearHeading(blueClosePostouttakePose)
                .build();

        TrajectorySequence blueCloseMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(270)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                .lineToLinearHeading(blueClosePostouttakePose)
                .build();

        TrajectorySequence blueCloseMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, 36, Math.toRadians(0)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                .lineToLinearHeading(blueClosePostouttakePose)
                .build();

        //Far Blue
        TrajectorySequence blueFarMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(180)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                //giving more lee-way to wall because bot was hitting wall during testing
                .lineToLinearHeading(blueFarPostouttakePose)
                .build();

        TrajectorySequence blueFarMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(blueFarStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(270)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                .lineToLinearHeading(blueFarPostouttakePose)
                .build();

        TrajectorySequence blueFarMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(blueCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(0)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                .lineToLinearHeading(blueFarPostouttakePose)
                .build();

        //Close red
        TrajectorySequence redCloseMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(180)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                //giving more lee-way to wall because bot was hitting wall during testing
                .lineToLinearHeading(redClosePostouttakePose)
                .build();

        TrajectorySequence redCloseMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(270)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                .lineToLinearHeading(redClosePostouttakePose)
                .build();

        TrajectorySequence redCloseMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(0)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                .lineToLinearHeading(redClosePostouttakePose)
                .build();

        //Far red
        TrajectorySequence redFarMarkerRightOuttakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(180)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                //giving more lee-way to wall because bot was hitting wall during testing
                .lineToLinearHeading(redFarPostouttakePose)
                .build();

        TrajectorySequence redFarMarkerMiddleOuttakeSequence = drive.trajectorySequenceBuilder(redFarStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(270)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                .lineToLinearHeading(redFarPostouttakePose)
                .build();

        TrajectorySequence redFarMarkerLeftOuttakeSequence = drive.trajectorySequenceBuilder(redCloseStartPose)
                //reach tile at correct heading
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(0)))
                //outtake of tile goes in this marker
                .addDisplacementMarker(() ->{outtakePlaceHolder();})
                //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
                .lineToLinearHeading(redFarPostouttakePose)
                .build();


        /*
        Drop yellow pixel in intake sequence
        because all outtakes return to the same spot the same drop sequence can be utilized for all sequences of same start pose
         */
        
        //Close Blue
        TrajectorySequence blueCloseDropSequence = drive.trajectorySequenceBuilder(blueClosePostouttakePose)
                .splineTo(blueBackDrop, Math.toRadians(0))
                .addDisplacementMarker(() -> {dropPlaceHolder();})
                .build();

        //Far Blue
        TrajectorySequence blueFarDropSequence = drive.trajectorySequenceBuilder(blueFarPostouttakePose)
                .lineToLinearHeading(blueClosePostouttakePose)
                .splineTo(blueBackDrop, Math.toRadians(0))
                .build();

        //Close red
        TrajectorySequence redCloseDropSequence = drive.trajectorySequenceBuilder(redClosePostouttakePose)
                .splineTo(redBackDrop, Math.toRadians(0))
                .addDisplacementMarker(() -> {dropPlaceHolder();})
                .build();

        //Far red
        TrajectorySequence redFarDropSequence = drive.trajectorySequenceBuilder(redFarPostouttakePose)
                .lineToLinearHeading(redClosePostouttakePose)
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




        waitForStart();

        if (!isStopRequested())
            //EXAMPLE SEQUENCE CHAIN
            drive.followTrajectorySequence(blueCloseMarkerLeftOuttakeSequence);
            drive.followTrajectorySequence(blueCloseDropSequence);
            drive.followTrajectorySequence(blueNearParkSequence);
    }
}
