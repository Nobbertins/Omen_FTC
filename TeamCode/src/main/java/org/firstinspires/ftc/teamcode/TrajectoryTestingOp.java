package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryTestingOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueCloseStartPose = new Pose2d(12,56,Math.toRadians(270));
        Pose2d blueFarStartPose = new Pose2d(-35,56,Math.toRadians(270));
        Pose2d redCloseStartPose = new Pose2d(12,-56,Math.toRadians(90));
        Pose2d redFarStartPose = new Pose2d(-35,-56,Math.toRadians(90));

        Pose2d startPose = blueCloseStartPose;

        drive.setPoseEstimate(startPose);

        TrajectorySequence autonomousDropSequence = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(46, 36), Math.toRadians(0))
                .waitSeconds(3)
                .build();

        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(autonomousDropSequence);
    }
}
