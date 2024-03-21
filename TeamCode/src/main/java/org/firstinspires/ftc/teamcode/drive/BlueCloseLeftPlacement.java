package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.Drive;
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


    //Initialize Motors and Servos, if you wish to control other actuators in a trajectory first intialize them here
    private DcMotor rraiseMotor = null;

    private DcMotor lraiseMotor = null;
    private Servo depositServo = null;

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
/*
    private TrajectorySequence closeTrajectoryBuild(Pose2d startPose, SampleMecanumDrive drive, Vector2d spikeStagingWaypoint, Vector2d spikeEndWaypoint,
                                                    Vector2d backdropStagingWaypoint, Vector2d backdropEndWaypoint,
                                                    Vector2d parkStagingWaypoint, Vector2d parkEndWaypoint){
        //Trajectory which is run if the piece is detected on the left spike mark
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .splineTo(spikeEndWaypoint, Math.toRadians(1))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1, ()->rslideServo.setPosition(0.24))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->slideStop())
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(0))

                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropEndWaypoint, Math.toRadians(180)), Math.toRadians(5))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(1,()->depositServo.setPosition(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideStop())

                //back off and prepare for park
                .splineToConstantHeading(parkStagingWaypoint, Math.toRadians(3))
                //lower slides
                .UNSTABLE_addTemporalMarkerOffset(1,()->rslideServo.setPosition(0.02))
                .UNSTABLE_addTemporalMarkerOffset(1,()->slideDrop())
                //park
                .splineToConstantHeading(parkEndWaypoint, Math.toRadians(3))
                .build();
        return trajectorySequence;
    }
*/
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
        rslideServo = hardwareMap.get(Servo.class, "rslide");
        //Intialize robot
        rslideServo.setPosition(0.02);
        //close deposit servo to lock in piece
        depositServo.setPosition(0);

        Vector2d leftSpikeEndWaypoint = new Vector2d(17,47);
        Vector2d middleSpikeEndWaypoint = new Vector2d(11,43);
        Vector2d rightSpikeEndWaypoint = new Vector2d(15, 41);

        Vector2d backdropStagingWaypoint = new Vector2d(30,54);


        Vector2d backdropLeftEndWaypoint = new Vector2d(50, 43);
        Vector2d backdropMiddleEndWaypoint = new Vector2d(50,37);
        Vector2d backdropRightEndWaypoint = new Vector2d(51,30);

        Vector2d parkStagingWaypoint = new Vector2d(42,60);
        Vector2d parkEndWaypoint = new Vector2d(54,60);


        //Starting position and heading of the robot. *YOU SHOULD NOT NEED TO CHANGE HEADING*
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Trajectory which is run if the piece is detected on the right spike mark
        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
            //drive forward to a better start locaiton
            .lineToConstantHeading(new Vector2d(15, 55))
            //Drive to Pixel
            .lineToLinearHeading(new Pose2d(rightSpikeEndWaypoint, Math.toRadians(0)))
            //could be wrong if marker offsets chain off eachother
            .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
            .UNSTABLE_addTemporalMarkerOffset(0.3, ()->slideStop())
            .UNSTABLE_addTemporalMarkerOffset(0.5, ()->rslideServo.setPosition(0.28))
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


        //Trajectory which is run if the piece is detected on the middle spike mark
        TrajectorySequence trajSeqMiddle = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineToLinearHeading(new Pose2d(middleSpikeEndWaypoint, Math.toRadians(70)))
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(0))

                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropMiddleEndWaypoint, Math.toRadians(180)), Math.toRadians(20), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //Drop pixel
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->depositServo.setPosition(0.5))
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



        //Trajectory which is run if the piece is detected on the left spike mark
        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                //Drive to Pixel
                .lineTo(leftSpikeEndWaypoint)
                //could be wrong if marker offsets chain off eachother
                .UNSTABLE_addTemporalMarkerOffset(0,()->slideRaise())
                .UNSTABLE_addTemporalMarkerOffset(0.8, ()->slideStop())
                .UNSTABLE_addTemporalMarkerOffset(0.9, ()->rslideServo.setPosition(0.28))
                //Leave pixel behind and begin route
                .splineToConstantHeading(backdropStagingWaypoint, Math.toRadians(0))

                //spline to back drop
                .splineToSplineHeading(new Pose2d(backdropLeftEndWaypoint, Math.toRadians(180)), Math.toRadians(0.1), SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
