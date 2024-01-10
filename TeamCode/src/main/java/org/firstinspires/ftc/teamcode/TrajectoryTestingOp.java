package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajectoryTestingOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueCloseStartPose = new Pose2d(12,61,Math.toRadians(270));
        Pose2d blueFarStartPose = new Pose2d(-35,61,Math.toRadians(270));
        Pose2d redCloseStartPose = new Pose2d(12,-61,Math.toRadians(90));
        Pose2d redFarStartPose = new Pose2d(-35,-61,Math.toRadians(90));

        Pose2d startPose = blueCloseStartPose;

        drive.setPoseEstimate(startPose);

        TrajectorySequence autonomousDropSequence = drive.trajectorySequenceBuilder(startPose)
                //to tile of deposit
                .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(180)))
                .waitSeconds(3)

                //back to start
                .lineToLinearHeading(startPose)
                //move to side to avoid running into marker on left line
                .strafeLeft(10)
                .splineToSplineHeading(new Pose2d(46, 36, Math.toRadians(0)), Math.toRadians(10))

                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(autonomousDropSequence);
    }
}
