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
public class BlueCloseLeftPlacement extends LinearOpMode {


    // pos x is closer to the wall in this case, pos y is farther away from wall
    public int pixelPlacement;//0 is left, 1 is right

    public BlueCloseLeftPlacement(){
        pixelPlacement = 0;
    }


    //Initialize Motors and Servos, if you wish to control other actuators in a trajectory first intialize them here
    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;
    private Servo depositServo = null;
    private Servo lslideServo = null;

    private Servo rslideServo = null;

    //Raise bucket to drop height
    private void slideRaise() {
        rraiseMotor.setDirection(DcMotor.Direction.REVERSE);
        lraiseMotor.setDirection(DcMotor.Direction.FORWARD);
        lraiseMotor.setPower(0.6);
        rraiseMotor.setPower(0.6);
    }

    //Keep slide motors running at a low power to lock them in place
    private void slideStop(){
        lraiseMotor.setPower(0.05);
        rraiseMotor.setPower(0.05);
    }

    //Bring bucket back down after pixel has been dropped
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

        //Intialize webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        //Pipeline is the
        webcam1.setPipeline(new BlueCloseLeftPlacement.examplePipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        //Intialize motors based on how they are listed listed in the robot configuration. This configuration can be edited from the dirver control hub
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        rraiseMotor = hardwareMap.get(DcMotor.class, "rraise");
        lraiseMotor = hardwareMap.get(DcMotor.class, "lraise");
        depositServo = hardwareMap.get(Servo.class, "deposit");
        lslideServo = hardwareMap.get(Servo.class, "lslide");
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        //Intialize robot
        rslideServo.setPosition(0.02);
        //close deposit servo to lock in piece
        depositServo.setPosition(0);

        //Starting position and heading of the robot. *YOU SHOULD NOT NEED TO CHANGE HEADING*
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Trajectory which is run if the piece is detected on the right spike mark
        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                //go out a little (better start point)
                .lineTo(new Vector2d(18, 50))
                //push pixel to the line by moving to a point while rotating the bot to the side
                .lineToLinearHeading(new Pose2d(12,40, Math.toRadians(0)))
                //move back from the line to let go of pixel on line
                .lineTo(new Vector2d(15, 40))

                //Raise the servo slide to prepare for backboard dropping
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.9)
                //flip the bucket out to be parellel to the backboard angle
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                //lock slides in this position
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)

                //drive to backboard while rotating robot to be facing backdrop
                //32 is the absolute right side of the back board, move slightly left by the (pixelPlacement * 1.2) term for accurate placement in the right column
                .lineToLinearHeading(new Pose2d(45, 32 - (pixelPlacement * 1.2), Math.toRadians(180)))
                //Go forward into the backboard until the bucket is right up against the backdrop
                .back(9)

                //Open the bucket and drop the pixel
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)

                //Raise slide to release pixel from bucket completly
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                //bring the bucket back in
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                //lower slides all the way down
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                //Back away from backboard
                .forward(5)
                //the robot will go relatively left all the way to corner
                .strafeRight(28 + (pixelPlacement * 3.2))
                //go forward into corner
                .back(8)
                .build();

        //Trajectory which is run if the piece is detected on the middle spike mark
        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //go out a little (better start point)
                .lineTo(new Vector2d(18, 55))
                //push pixel to line
                .lineToLinearHeading(new Pose2d(27, 28.5, Math.toRadians(0)))
                //leave pixel on line
                .lineTo(new Vector2d(35, 28.5))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.9)
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(45, 37 - (pixelPlacement * 2.2), Math.toRadians(180)))
                .back(9)
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                // added forward to make sure the bot moves forward away from the wall
                .forward(5)
                .strafeRight(22 + (pixelPlacement * 3.2))
                .back(8)
                .build();

        //Trajectory which is run if the piece is detected on the left spike mark
        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(18, 43))
                .lineTo(new Vector2d(18, 50))
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.9)
                .addTemporalMarker(()->rslideServo.setPosition(0.24))
                .addTemporalMarker(()->slideStop())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(45, 45 - (pixelPlacement * 3.2), Math.toRadians(180)))
                .back(9)
                .addTemporalMarker(()->depositServo.setPosition(0.5))
                .waitSeconds(1)
                .addTemporalMarker(()->slideRaise())
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideStop())
                .addTemporalMarker(()->rslideServo.setPosition(0.02))
                .waitSeconds(0.4)
                .addTemporalMarker(()->slideDrop())
                .waitSeconds(1)
                .forward(5)
                .strafeRight(15 + (pixelPlacement * 3.2))
                .back(8)
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


    /*
    Pipelines are how you control piece detection. At the start the camera can only view two of the three spike marks.
    This pipeline works by splitting the camera view into two sides by making two crops. Then the blue channel (how much blue there is)
    is extracted from each crop and the difference between the blue between the two crops is taken. If this difference is above a certain
    sensitivty threshold, the piece is considered to be on the side with the higher blue content. If the sensitivity
    threshold is not exceeded then the piece is considered to be on the line that is not in the view.

    Settings You Can Alter
        -Sensitivity of Color Detection
        -Size of the two crops

    Camera lens was 12.5cm from right side of tile and at edge of tile when these values were calibrated
     */
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Double leftavgin;
        Double rightavgin;
        Mat outPut = new Mat();

        /*
        The crops which are compared
        The x and y arguments are the root vertex of the rectangle
        Width is the amount the rectangle is expanded to the right
        Hieght is the amount the rectangle is expanded down

        *Important* (0,0) is the top left corner so a higher y value will raise the rectangle crop up
        x + width must be less than 640 as this is the x-resolution of the webcam
        y + height must be less than 360 as this is the y-resolution of the webcam
         */
        Rect leftRect = new Rect(100, 220, 160, 139);

        Rect rightRect = new Rect(420, 220, 160, 139);

        //red = 1, blue = 2
        int color = 2;


        //The sensitivty threshold: Lower sensitivity value means more likely to detect a piece on a visible spike but less likely to detect if the piece is placed on a non-visible spike
        double sensitivity = 5.0;

        /*
        position of left, right, middle are determined as the left line being the line closest to the backdrop
        */

        @Override
        //*YOU SHOULD NOT NEED TO CHNAGE ANYTHING IN THIS FUNCTION FOR TUNING*
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
            //when viewing just right and middle lines
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

            //Print blue values of each crop and the predicted position on the driver controller
            telemetry.addData("Leftavgin", leftavgin);
            telemetry.addData("Rightavgin", rightavgin);
            telemetry.addData("Marker Position", currentMarkerPosition);
            telemetry.update();

            return(outPut);

        }
    }
}
