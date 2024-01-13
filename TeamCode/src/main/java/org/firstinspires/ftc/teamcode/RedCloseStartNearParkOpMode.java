package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class RedCloseStartNearParkOpMode extends OpMode {

    SampleMecanumDrive drive = null;
    OpenCvWebcam webcam1 = null;
    RedMarkerDetectionPipeline workingPipeline = new RedMarkerDetectionPipeline();

    //drive drive = new drive();

    public void sleepMillis(int delay){
        Long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < delay){}
    }

    @Override
    public void init(){

        drive = new SampleMecanumDrive(hardwareMap);


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        webcam1.setPipeline(workingPipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera Init");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera err", errorCode);
                telemetry.update();
            }

        });

    }


        public void start() {
//Determine location of marker and outtake the purple pixel
            Pose2d startPose =  new Pose2d(14,-61,Math.toRadians(270));
            MarkerPosition position = workingPipeline.getCurrentMarkerPosition();
            telemetry.addData("Line Detected", position);
            telemetry.update();
            //Set start position estimate for drive to avoid mess ups in follower
            drive.setPoseEstimate(startPose);



            switch(position){
                case LEFT:
                    /*
                    drive.followTrajectorySequence(drive.blueCloseMarkerLeftOuttakeSequence);
                    drive.setPoseEstimate(drive.blueClosePostOuttakePose);
                    drive.followTrajectorySequence(drive.blueCloseDropLeftSequence);
                    //park near
                    drive.followTrajectorySequence(drive.blueLeftNearParkSequence);
                    */
                    drive.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(14-4.5, -61+20))
                            .forward(18)
                            //Outtake of tile goes in this marker
                            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                            .strafeLeft(50)
                            .build();
                    break;

                case RIGHT:
                    /*
                    drive.followTrajectorySequence(drive.blueCloseMarkerRightOuttakeSequence);
                    drive.setPoseEstimate(drive.blueClosePostOuttakePose);

                    drive.followTrajectorySequence(drive.blueCloseDropRightSequence);
                    //park near
                    drive.followTrajectorySequence(drive.blueRightNearParkSequence);
                    */
                    drive.trajectorySequenceBuilder(startPose)
                            .lineTo(new Vector2d(14+14, -61+20))
                            .forward(18)
                            //Outtake of tile goes in this marker
                            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                            .strafeLeft(32)
                            .build();
                    break;

                case MIDDLE:
                    /*
                    drive.followTrajectorySequence(drive.blueCloseMarkerMiddleOuttakeSequence);
                    drive.setPoseEstimate(drive.blueClosePostOuttakePose);

                    drive.followTrajectorySequence(drive.blueCloseDropMiddleSequence);
                    //park near
                    drive.followTrajectorySequence(drive.blueMiddleNearParkSequence);
                    */
                    drive.trajectorySequenceBuilder(startPose)
                            .back(30)
                            .forward(30)
                            //Outtake of tile goes in this marker
                            //return to start to be able to use the same dropSequence for all blue close starts but facing backdrop
                            .strafeRight(45)
                            .build();
                    break;

                default:
                    telemetry.addLine("error with camera");
                    //stop();
                    break;
            }



    }

    @Override
    public void loop() {

    }

}