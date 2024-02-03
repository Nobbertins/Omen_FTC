package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajectoryTestRedFarAuto extends LinearOpMode {


    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;
    private Servo depositServo = null;
    private Servo lslideServo = null;

    private Servo rslideServo = null;
    private void slideRaise() {
        rraiseMotor.setDirection(DcMotor.Direction.REVERSE);
        lraiseMotor.setDirection(DcMotor.Direction.FORWARD);
        lraiseMotor.setPower(0.6);
        rraiseMotor.setPower(0.6);
    }
    private void swingArm() {
        rslideServo.setPosition(0.24);
        lraiseMotor.setPower(0.05);
        rraiseMotor.setPower(0.05);
    }
    public void slideDrop() {
        rslideServo.setPosition(0.02);
        rraiseMotor.setDirection(DcMotor.Direction.FORWARD);
        lraiseMotor.setDirection(DcMotor.Direction.REVERSE);
        lraiseMotor.setPower(0.5);
        rraiseMotor.setPower(0.5);
    }
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rraiseMotor = hardwareMap.get(DcMotor.class, "rraise");
        lraiseMotor = hardwareMap.get(DcMotor.class, "lraise");
        depositServo = hardwareMap.get(Servo.class, "deposit");
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        rslideServo.setPosition(0.02);
        depositServo.setPosition(0);
        Pose2d startPose = new Pose2d(-38, -62, Math.toRadians(270));
drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqFarLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-40, -50))
                .lineToLinearHeading(new Pose2d(-36, -40, Math.toRadians(0)))
                .lineTo(new Vector2d(-32, -40))
                .lineTo(new Vector2d(-32, 0))
                .lineTo(new Vector2d(15, 0))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(1.2)
                .addTemporalMarker(()->swingArm())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49, -23, Math.toRadians(180)))
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                .strafeLeft(37)
                .back(11)
                .build();
        TrajectorySequence trajSeqFarMiddle = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-38, -34))
                .lineTo(new Vector2d(-38, -60))
                .lineTo(new Vector2d(-54, -20))
                .lineTo(new Vector2d(-32, 0))
                .lineTo(new Vector2d(15, 0))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(1.2)
                .addTemporalMarker(()->swingArm())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49, -23, Math.toRadians(180)))
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                .strafeLeft(37)
                .back(11)
                .build();
        TrajectorySequence trajSeqFarRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(17, -40))
                .lineTo(new Vector2d(17, -50))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(1.2)
                .addTemporalMarker(()->swingArm())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(49, -45, Math.toRadians(180)))
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                .strafeLeft(15)
                .back(11)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeqFarMiddle);
    }
}