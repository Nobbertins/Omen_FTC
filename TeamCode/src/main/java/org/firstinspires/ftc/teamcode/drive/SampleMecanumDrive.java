package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(6, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "fl");
        leftRear = hardwareMap.get(DcMotorEx.class, "bl");
        rightRear = hardwareMap.get(DcMotorEx.class, "br");
        rightFront = hardwareMap.get(DcMotorEx.class, "fr");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));
        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
    public void OuttakePlaceHolder(){

    }

    //Fill this function with the functionality to drop off a pixel on the backdrop
    public void dropPlaceHolder(){

    }
    //IMPORTANT LOCATIONS

    //I AM NOT CHANGING ANY REDS AT THIS TIME
    public Pose2d blueCloseStartPose = new Pose2d(14,61,Math.toRadians(90));
    public Pose2d blueFarStartPose = new Pose2d(-36,61,Math.toRadians(90));
    public Pose2d redCloseStartPose = new Pose2d(14,-61,Math.toRadians(270));
    public Pose2d redFarStartPose = new Pose2d(-36,-61,Math.toRadians(270));

    //

    public Pose2d blueClosePostOuttakePose = new Pose2d(12,58,Math.toRadians(0));
    public Pose2d blueFarPostOuttakePose = new Pose2d(-36,58,Math.toRadians(0));
    public Pose2d redClosePostOuttakePose = new Pose2d(12,-58,Math.toRadians(0));
    public Pose2d redFarPostOuttakePose = new Pose2d(-36,-58,Math.toRadians(0));


    public Vector2d blueBackDropLeft = new Vector2d(48,40);
    public Vector2d redBackDropLeft = new Vector2d(48, -40);

    public Vector2d blueBackDropMiddle = new Vector2d(48,35);
    public Vector2d redBackDropMiddle = new Vector2d(48, -35);

    public Vector2d blueBackDropRight = new Vector2d(48,30);
    public Vector2d redBackDropRight = new Vector2d(48, -30);



    public Vector2d blueNearPark = new Vector2d(60,60);
    public Vector2d blueAwayPark = new Vector2d(60,12);
    public Vector2d redNearPark = new Vector2d(60,-60);
    public Vector2d redAwayPark = new Vector2d(60,-12);

    public Vector2d blueCloseMarkerTile = new Vector2d(12, 36);
    public Vector2d blueFarMarkerTile = new Vector2d(-36, 36);
    public Vector2d redCloseMarkerTile = new Vector2d(12, -36);
    public Vector2d redFarMarkerTile = new Vector2d(-36, -36);

    //neccesary to prevent mismatch between the driver's representation of pose and the trajectory's
    //only need to do this once for the start pose in the beginning if you chain trajectory sequences

    //All trajectories for automatic Outtake of pixel on marker's line

    //Close Blue
    public TrajectorySequence blueCloseMarkerRightOuttakeSequence = this.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            //TESTING A SPLINE HEADING

            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    public TrajectorySequence blueCloseMarkerMiddleOuttakeSequence = this.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading
            //TESTING A SPLINE HEADING

            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(90)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    public TrajectorySequence blueCloseMarkerLeftOuttakeSequence = this.trajectorySequenceBuilder(blueCloseStartPose)
            //reach tile at correct heading

            //TESTING A SPLINE HEADING
            .lineToSplineHeading(new Pose2d(12, 36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueClosePostOuttakePose)
            .build();

    //Far Blue
    public TrajectorySequence blueFarMarkerRightOuttakeSequence = this.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    public TrajectorySequence blueFarMarkerMiddleOuttakeSequence = this.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    public TrajectorySequence blueFarMarkerLeftOuttakeSequence = this.trajectorySequenceBuilder(blueFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
            .lineToLinearHeading(blueFarPostOuttakePose)
            .build();

    //Close red
    public TrajectorySequence redCloseMarkerRightOuttakeSequence = this.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    public TrajectorySequence redCloseMarkerMiddleOuttakeSequence = this.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    public TrajectorySequence redCloseMarkerLeftOuttakeSequence = this.trajectorySequenceBuilder(redCloseStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(12, -36, Math.toRadians(0)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redClosePostOuttakePose)
            .build();

    //Far red
    public TrajectorySequence redFarMarkerRightOuttakeSequence = this.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(180)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            //giving more lee-way to wall because bot was hitting wall during testing
            .lineToLinearHeading(redFarPostOuttakePose)
            .build();

    public TrajectorySequence redFarMarkerMiddleOuttakeSequence = this.trajectorySequenceBuilder(redFarStartPose)
            //reach tile at correct heading
            .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(270)))
            //Outtake of tile goes in this marker
            .addDisplacementMarker(() ->{OuttakePlaceHolder();})
            //return to start to be able to use the same dropSequence for all red close starts but facing backdrop
            .lineToLinearHeading(redFarPostOuttakePose)
            .build();

    public TrajectorySequence redFarMarkerLeftOuttakeSequence = this.trajectorySequenceBuilder(redFarStartPose)
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
    public TrajectorySequence blueCloseDropMiddleSequence = this.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    public TrajectorySequence blueCloseDropRightSequence = this.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    public TrajectorySequence blueCloseDropLeftSequence = this.trajectorySequenceBuilder(blueClosePostOuttakePose)
            .splineTo(blueBackDropLeft, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();


    //Far Blue
    public TrajectorySequence blueFarDropMiddleSequence = this.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropMiddle, Math.toRadians(0))
            .build();
    public TrajectorySequence blueFarDropRightSequence = this.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropRight, Math.toRadians(0))
            .build();
    public TrajectorySequence blueFarDropLeftSequence = this.trajectorySequenceBuilder(blueFarPostOuttakePose)
            .lineToLinearHeading(blueClosePostOuttakePose)
            .splineTo(blueBackDropLeft, Math.toRadians(0))
            .build();


    //Close red
    public TrajectorySequence redCloseDropMiddleSequence = this.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    public TrajectorySequence redCloseDropRightSequence = this.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    public TrajectorySequence redCloseDropLeftSequence = this.trajectorySequenceBuilder(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Far red
    public TrajectorySequence redFarDropMiddleSequence = this.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropMiddle, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();
    public TrajectorySequence redFarDropRightSequence = this.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropRight, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    public TrajectorySequence redFarDropLeftSequence = this.trajectorySequenceBuilder(redFarPostOuttakePose)
            .lineToLinearHeading(redClosePostOuttakePose)
            .splineTo(redBackDropLeft, Math.toRadians(0))
            .addDisplacementMarker(() -> {dropPlaceHolder();})
            .build();

    //Trajectories for parking after dropping off pixel, they are the same for all blue starts and all red starts

    //Blue Near
    public TrajectorySequence blueNearParkSequence = this.trajectorySequenceBuilder(new Pose2d(blueBackDropMiddle, Math.toRadians(0)))
            .strafeLeft(23)
            .forward(12)
            .build();
    //Blue Away
    public TrajectorySequence blueAwayParkSequence = this.trajectorySequenceBuilder(new Pose2d(blueBackDropMiddle, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeRight(24)
            .forward(12)
            .build();
    //Red Near
    public TrajectorySequence redNearParkSequence = this.trajectorySequenceBuilder(new Pose2d(redBackDropMiddle, Math.toRadians(0)))
            .strafeRight(23)
            .forward(12)
            .build();
    //Red Away
    public TrajectorySequence redAwayParkSequence = this.trajectorySequenceBuilder(new Pose2d(redBackDropMiddle, Math.toRadians(0)))
            //slightly longer than near strafe to make SURE that the backdrop is not hit
            .strafeLeft(24)
            .forward(12)
            .build();

}
