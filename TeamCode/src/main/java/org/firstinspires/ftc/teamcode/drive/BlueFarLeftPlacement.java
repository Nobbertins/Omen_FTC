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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;


// btw strafeRight goes left and strafeLeft goes right
@Autonomous
public class BlueFarLeftPlacement extends LinearOpMode {

    public int pixelPlacement;//0 is left, 1 is right

    public BlueFarLeftPlacement(){
        pixelPlacement = 0;
    }
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
    private void slideStop(){
        lraiseMotor.setPower(0.05);
        rraiseMotor.setPower(0.05);
    }
    public void slideDrop() {
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
        webcam1.setPipeline(new BlueFarLeftPlacement.examplePipeline());
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

        Vector2d rightSpikeEndWaypoint = new Vector2d(-40,44);
        Vector2d rightSpikeStagingWaypoint = new Vector2d(-36, 55);
        Vector2d middleSpikeEndWaypoint = new Vector2d(-37,36);

        Vector2d leftSpikeEndWaypoint = new Vector2d(-45, 51);
        Vector2d leftSpikeStagingWaypoint = new Vector2d(-30,55);
        Vector2d trussStagingWaypoint = new Vector2d(-35, 60);
        Vector2d trussEndWaypoint = new Vector2d(-12,60);

        Vector2d backdropStagingWaypoint = new Vector2d(30,54);


        Vector2d backdropLeftEndWaypoint = new Vector2d(50, 43);
        Vector2d backdropMiddleEndWaypoint = new Vector2d(51,33.5);
        Vector2d backdropRightEndWaypoint = new Vector2d(51,29);

        Vector2d parkStagingWaypoint = new Vector2d(42,10);
        Vector2d parkEndWaypoint = new Vector2d(54,10);

        Pose2d startPose = new Pose2d(-33, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                //drive to spike
                .lineTo(leftSpikeEndWaypoint)
                //Drive through truss
                .lineTo(trussStagingWaypoint)
                .lineTo(trussEndWaypoint)
                .waitSeconds(0.3)
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.9, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropRightEndWaypoint, Math.toRadians(180)), Math.toRadians(310), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //drop pixel
                .lineToLinearHeading(new Pose2d(middleSpikeEndWaypoint, Math.toRadians(110)),  SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //drive through truss
                .lineToSplineHeading(new Pose2d(trussStagingWaypoint, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(trussEndWaypoint)
                .waitSeconds(1)
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1.8, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(1.9, ()->rslideServo.setPosition(0.28))
                .lineTo(backdropStagingWaypoint)
                //Leave pixel behind and begin route
                //spline to back drop
                .splineToConstantHeading(backdropMiddleEndWaypoint, Math.toRadians(290), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                //drop pixel
                .lineToConstantHeading(rightSpikeEndWaypoint, SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //drive through truss
                .lineToLinearHeading(new Pose2d(trussStagingWaypoint, Math.toRadians(180)),SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToConstantHeading(trussEndWaypoint)
                .waitSeconds(1)
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(2, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(2.1, ()->rslideServo.setPosition(0.28))
                .lineTo(backdropStagingWaypoint)
                //Leave pixel behind and begin route
                //spline to back drop
                .splineToConstantHeading(backdropRightEndWaypoint, Math.toRadians(290), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
        Rect leftRect = new Rect(150, 230, 190, 129);
        // y = 80 og
        Rect rightRect = new Rect(450, 230, 190, 129);
        // y 95 og
        // x 300 og
        int color = 2;
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
