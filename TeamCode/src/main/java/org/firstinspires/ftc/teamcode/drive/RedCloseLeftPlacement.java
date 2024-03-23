package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MarkerPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class RedCloseLeftPlacement extends LinearOpMode {
    private DcMotor rraiseMotor = null;
private DcMotor intakeMotor = null;
    private DcMotor lraiseMotor = null;
    private Servo ldropServo = null;
    private Servo rdropServo = null;
    private Servo depositServo = null;

    private Servo rslideServo = null;

    private void slideStop(){
        lraiseMotor.setPower(0.05);
        rraiseMotor.setPower(0.05);
    }

    private void slideRaise() {
        rraiseMotor.setDirection(DcMotor.Direction.REVERSE);
        lraiseMotor.setDirection(DcMotor.Direction.FORWARD);
        lraiseMotor.setPower(0.6);
        rraiseMotor.setPower(0.6);
    }
    public void slideDrop() {
        rslideServo.setPosition(0.02);
        rraiseMotor.setDirection(DcMotor.Direction.FORWARD);
        lraiseMotor.setDirection(DcMotor.Direction.REVERSE);
        lraiseMotor.setPower(0.5);
        rraiseMotor.setPower(0.5);
    }
    private void runIntake(){
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(1);
    }
    private void stopIntake(){
        intakeMotor.setPower(0);
    }
    private void runOuttake(){
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setPower(0.9);
    }
    private void moveIntake(double position){
        //from 0 (top) to 0.5 (bottom)
        ldropServo.setPosition(0.5 - position);
        rdropServo.setPosition(position);
    }
    OpenCvWebcam webcam1 = null;
    public MarkerPosition currentMarkerPosition = MarkerPosition.UNKNOWN;
    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        webcam1.setPipeline(new RedCloseLeftPlacement.examplePipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rraiseMotor = hardwareMap.get(DcMotor.class, "rraise");
        lraiseMotor = hardwareMap.get(DcMotor.class, "lraise");
        depositServo = hardwareMap.get(Servo.class, "deposit");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        rdropServo = hardwareMap.get(Servo.class, "rdrop");
        ldropServo = hardwareMap.get(Servo.class, "ldrop");
        rslideServo.setPosition(0.03);
        depositServo.setPosition(0);
        moveIntake(0);
        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Vector2d rightSpikeEndWaypoint = new Vector2d(17,-47);
        Vector2d middleSpikeEndWaypoint = new Vector2d(11,-37);
        Vector2d leftSpikeEndWaypoint = new Vector2d(11.5, -40);

        Vector2d backdropStagingWaypoint = new Vector2d(30,-54);

        Vector2d backdropLeftEndWaypoint = new Vector2d(51, -31);
        Vector2d backdropMiddleEndWaypoint = new Vector2d(51,-37);
        Vector2d backdropRightEndWaypoint = new Vector2d(51,-42.8);

        Vector2d parkStagingWaypoint = new Vector2d(42,-60);
        Vector2d parkEndWaypoint = new Vector2d(56,-60);


        //Starting position and heading of the robot. *YOU SHOULD NOT NEED TO CHANGE HEADING*
        drive.setPoseEstimate(startPose);

        //Trajectory which is run if the piece is detected on the right spike mark
        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToConstantHeading(rightSpikeEndWaypoint, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.81, ()->slideStop())
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropRightEndWaypoint, Math.toRadians(180)), Math.toRadians(310), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0,()->rslideServo.setPosition(0.28))
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1.3,()->rslideServo.setPosition(0.03))
                .UNSTABLE_addTemporalMarkerOffset(1.8,()->slideDrop())
                .waitSeconds(2)
                .forward(3)
                .strafeRight(30)
                .back(8)
                .build();
        //Trajectory which is run if the piece is detected on the middle spike mark
        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToLinearHeading(new Pose2d(middleSpikeEndWaypoint, Math.toRadians(300)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropMiddleEndWaypoint, Math.toRadians(180)), Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->rslideServo.setPosition(0.03))
                .UNSTABLE_addTemporalMarkerOffset(1.6,()->slideDrop())
                .waitSeconds(2)
                .forward(3)
                .strafeRight(20)
                .back(8)
                .build();



        //Trajectory which is run if the piece is detected on the left spike mark
        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToLinearHeading(new Pose2d(leftSpikeEndWaypoint, Math.toRadians(359)), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.35, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.45, ()->rslideServo.setPosition(0.28))
                .waitSeconds(1.5)
                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropLeftEndWaypoint, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.95,()->rslideServo.setPosition(0.03))
                .UNSTABLE_addTemporalMarkerOffset(1.35,()->slideDrop())
                .waitSeconds(2)
                .forward(3)
                .strafeRight(10)
                .back(8)
                .build();

        waitForStart();
        Pose2d intakePose = null;
        if(isStopRequested()) return;
        int pushes = 0;
        double yPlacement = 0;
        switch(currentMarkerPosition){
            case LEFT:
                drive.followTrajectorySequence(trajSeqLeft);
                intakePose = trajSeqLeft.end();
                pushes = 1;
                yPlacement = backdropRightEndWaypoint.getY();
                break;
            case RIGHT:
                drive.followTrajectorySequence(trajSeqRight);
                intakePose = trajSeqRight.end();
                yPlacement = backdropLeftEndWaypoint.getY();
                break;
            case MIDDLE:
                drive.followTrajectorySequence(trajSeqMiddle);
                intakePose = trajSeqMiddle.end();
                yPlacement = backdropRightEndWaypoint.getY();
                break;
            default:
                break;
        }
        //Trajectory which is run if the piece is detected on the right spike mark
//        TrajectorySequence Intake = drive.trajectorySequenceBuilder(intakePose)
//                .splineTo(new Vector2d(20, -10),Math.toRadians(180))
//                .lineToLinearHeading(new Pose2d(-50, -10,Math.toRadians(180)))
//                .forward(6.5 + pushes)
//                //pickup pixels
//                .build();
//        drive.followTrajectorySequence(Intake);
//        depositServo.setPosition(0.5);
//        sleep(500);
//        moveIntake(0.5);
//        sleep(800);
//        drive.setMotorPowers(-0.4,-0.4,-0.4,-0.4);
//        sleep(800);
//        runIntake();
//        drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
//        sleep(600);
//        for(int i = 0; i < 3; i++) {
//            drive.setMotorPowers(0.3, 0.3, 0.3, 0.3);
//            sleep(200);
//            drive.setMotorPowers(0,0,0,0);
//            sleep(200);
//        }
//        sleep(1800);
//        stopIntake();
//        depositServo.setPosition(0);
//        sleep(500);
//        moveIntake(0);
//        runOuttake();
//        sleep(1500);
//        stopIntake();
//        TrajectorySequence Deposit = drive.trajectorySequenceBuilder(Intake.end())
//                .lineTo(new Vector2d(30, -10))
//                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
//                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->slideStop())
//                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->rslideServo.setPosition(0.28))
//                .lineTo(new Vector2d(50, yPlacement))
//                .addTemporalMarker(() -> depositServo.setPosition(0.5))
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> slideRaise())
//                .waitSeconds(0.7)
//                .addTemporalMarker(() -> slideStop())
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> rslideServo.setPosition(0.03))
//                .waitSeconds(0.5)
//                .build();
//        drive.followTrajectorySequence(Deposit);
    }
    //Camera lens was 12.5cm from right side of tile and at edge of tile when these values were calibrated
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Double leftavgin;
        Double rightavgin;
        Mat outPut = new Mat();

        //Currently crops only include the bottom third of the image as current camera position only ever sees marker in bottom
        Rect leftRect = new Rect(120, 200, 160, 159);

        Rect rightRect = new Rect(420, 200, 200, 159);

        int color = 1;
        //red = 1, blue = 2


        boolean sampling = true;

        /*
        position of left, right, middle are determined as the left line being the line closest to the backdrop
        */

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            input.copyTo(outPut);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);


            Core.extractChannel(leftCrop, leftCrop, color);
            Core.extractChannel(rightCrop, rightCrop, color);

            Imgproc.rectangle(outPut, leftRect, new Scalar(255.0,255.0,255.0),2);
            Imgproc.rectangle(outPut, rightRect, new Scalar(255.0,255.0,255.0),2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgin = leftavg.val[0];
            rightavgin = rightavg.val[0];
            //lower -> more sensitive to choosing left or right
            double sensitivity = 5.0;
            //when viewing just right and middle lines
            if (sampling){
                if (leftavgin - rightavgin < sensitivity && leftavgin - rightavgin > -sensitivity) {
                    //nothing detected must be in furthest left lane
                    currentMarkerPosition = MarkerPosition.LEFT;
                } else if (leftavgin > rightavgin) {
                    //on the left of screen which is middle line
                    currentMarkerPosition = MarkerPosition.MIDDLE;
                } else {
                    //on the right
                    currentMarkerPosition = MarkerPosition.RIGHT;
                }
            }



            telemetry.addData("Leftavgin", leftavgin);
            telemetry.addData("Rightavgin", rightavgin);
            telemetry.addData("Marker Position", currentMarkerPosition);
            telemetry.update();

            return(outPut);

        }
    }
}
