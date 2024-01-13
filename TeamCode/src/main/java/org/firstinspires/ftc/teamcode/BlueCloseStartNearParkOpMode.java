package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class BlueCloseStartNearParkOpMode extends OpMode {

    SampleMecanumDrive drive = null;
    OpenCvWebcam webcam1 = null;
    BlueMarkerDetectionPipeline workingPipeline = new BlueMarkerDetectionPipeline();

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

            MarkerPosition position = workingPipeline.getCurrentMarkerPosition();
            telemetry.addData("Line Detected", position);
            telemetry.update();
            //Set start position estimate for drive to avoid mess ups in follower
            drive.setPoseEstimate(drive.blueCloseStartPose);

            switch(position){
                case LEFT:
                    drive.followTrajectorySequence(drive.blueCloseMarkerLeftOuttakeSequence);
                    drive.followTrajectorySequence(drive.blueCloseDropLeftSequence);
                    break;

                case RIGHT:
                    drive.followTrajectorySequence(drive.blueCloseMarkerRightOuttakeSequence);
                    drive.followTrajectorySequence(drive.blueCloseDropRightSequence);
                    break;

                case MIDDLE:
                    drive.followTrajectorySequence(drive.blueCloseMarkerMiddleOuttakeSequence);
                    drive.followTrajectorySequence(drive.blueCloseDropMiddleSequence);
                    break;

                default:
                    telemetry.addLine("error with camera");
                    //stop();
                    break;
            }


            //park near
            drive.followTrajectorySequence(drive.blueNearParkSequence);

    }

    @Override
    public void loop() {

    }

}