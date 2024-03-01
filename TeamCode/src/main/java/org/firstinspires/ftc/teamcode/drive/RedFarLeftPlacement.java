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

@Autonomous
public class RedFarLeftPlacement extends LinearOpMode {

    public int pixelPlacement;//0 is left, 1 is right

    public RedFarLeftPlacement(){
        pixelPlacement = 0;
    }
    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;
    private Servo depositServo = null;
    private Servo lslideServo = null;

    private Servo rslideServo = null;
    private void printLocation(SampleMecanumDrive d){
        Pose2d poseEstimate = d.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }
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
        webcam1.setPipeline(new RedFarLeftPlacement.examplePipeline());
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
        Pose2d startPose = new Pose2d(-12-11, -62-0.5, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-18-11, -43-0.5))
                .addTemporalMarker(()->printLocation(drive))
                .lineTo(new Vector2d(-18-11, -55-0.5))
                .strafeLeft(7)
                .lineTo(new Vector2d(-20-11, -11-0.5))
                .lineToLinearHeading(new Pose2d(40-11, -18-0.5, Math.toRadians(180)))
                .addTemporalMarker(()->printLocation(drive))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.6)
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)
                .lineTo(new Vector2d( 64-11,-30-0.5-(pixelPlacement*3.5)))
                .back(9)
                .addTemporalMarker(()->printLocation(drive))
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideDrop())
                .addTemporalMarker(()->printLocation(drive))
                .waitSeconds(2)
                .build();
        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //go out a little (better start point)
                .lineTo(new Vector2d(-18, -55))
                //push pixel to line
                .lineToLinearHeading(new Pose2d(-29, -28.5, Math.toRadians(180)))
                //leave pixel on line
                .lineTo(new Vector2d(-33, -28.5))
                .strafeRight(18)
                .back(85)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.6)
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(62, -36-(pixelPlacement*3), Math.toRadians(180)))
                .back(8.5)
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(2)
                .build();
        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-18, -50))
                .lineToLinearHeading(new Pose2d(-12,-38, Math.toRadians(180)))
                .forward(7)
                .strafeRight(28)
                .back(75)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.6)
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)
                .lineTo(new Vector2d(62, -41.5-(pixelPlacement*2)))
                .back(9.5)
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(2)
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
        Rect leftRect = new Rect(70, 200, 200, 159);

        Rect rightRect = new Rect(360, 200, 200, 159);

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
