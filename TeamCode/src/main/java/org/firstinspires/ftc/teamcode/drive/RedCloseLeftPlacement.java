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

    public int pixelPlacement;//0 is left, 1 is right

    public RedCloseLeftPlacement(){
        pixelPlacement = 0;
    }
    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;
    private Servo depositServo = null;
    private Servo lslideServo = null;

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
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        rslideServo.setPosition(0.02);
        depositServo.setPosition(0);

        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Vector2d rightSpikeEndWaypoint = new Vector2d(15,-50);
        Vector2d middleSpikeEndWaypoint = new Vector2d(11,-38);
        Vector2d leftSpikeEndWaypoint = new Vector2d(11, -42);

        Vector2d backdropStagingWaypoint = new Vector2d(30,-54);

        Vector2d backdropLeftEndWaypoint = new Vector2d(50, -30);
        Vector2d backdropMiddleEndWaypoint = new Vector2d(51,-37);
        Vector2d backdropRightEndWaypoint = new Vector2d(51,-41.8);

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
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.9, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(90), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropRightEndWaypoint, Math.toRadians(180)), Math.toRadians(310), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> slideStop())
                .waitSeconds(1.5)
                //.UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                //.UNSTABLE_addTemporalMarkerOffset(1.5,()->slideStop())
                //back off and prepare for park
                .lineTo(parkStagingWaypoint)
                //lower slides
                .UNSTABLE_addTemporalMarkerOffset(0,()->rslideServo.setPosition(0.02))
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->slideDrop())
                //park
                .lineTo(parkEndWaypoint)
                .build();


        //Trajectory which is run if the piece is detected on the middle spike mark
        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToLinearHeading(new Pose2d(middleSpikeEndWaypoint, Math.toRadians(300)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.4, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropMiddleEndWaypoint, Math.toRadians(180)), Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1.5,()->slideStop())
                .waitSeconds(1)
                //back off and prepare for park
                .lineTo(parkStagingWaypoint)
                //lower slides
                .UNSTABLE_addTemporalMarkerOffset(0,()->rslideServo.setPosition(0.02))
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->slideDrop())
                //park
                .lineTo(parkEndWaypoint)
                .build();



        //Trajectory which is run if the piece is detected on the left spike mark
        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToLinearHeading(new Pose2d(leftSpikeEndWaypoint, Math.toRadians(359)), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.6, ()->rslideServo.setPosition(0.28))
                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropLeftEndWaypoint, Math.toRadians(180)), Math.toRadians(0), SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> slideStop())
                .waitSeconds(1)
                //.UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                //.UNSTABLE_addTemporalMarkerOffset(1.5,()->slideStop())
                //back off and prepare for park
                .lineTo(parkStagingWaypoint)
                //lower slides
                .UNSTABLE_addTemporalMarkerOffset(0,()->rslideServo.setPosition(0.02))
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->slideDrop())
                //park
                .lineTo(parkEndWaypoint)
                .build();

        waitForStart();

        if(isStopRequested()) return;
        switch(currentMarkerPosition){
            case LEFT:
                drive.followTrajectorySequence(trajSeqLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(trajSeqRight);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(trajSeqMiddle);
                break;
            default:
                break;
        }
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

        Rect rightRect = new Rect(420, 200, 160, 159);

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
