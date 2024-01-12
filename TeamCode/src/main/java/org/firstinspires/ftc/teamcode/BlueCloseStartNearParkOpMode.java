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
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    OpenCvWebcam webcam1 = null;
    BlueMarkerDetectionPipeline workingPipeline = new BlueMarkerDetectionPipeline();

    PlannedTrajectories plannedTrajectories = new PlannedTrajectories(drive);

    public void sleepMillis(int delay){
        Long start = System.currentTimeMillis();
        while(System.currentTimeMillis() - start < delay){}
    }

    @Override
    public void init(){

        //Set start position estimate for drive to avoid fuck ups in follower
        drive.setPoseEstimate(plannedTrajectories.blueCloseStartPose);


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        webcam1.setPipeline(workingPipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        sleepMillis(1000);

        //Determine location of marker and outtake the purple pixel
        MarkerPosition position = workingPipeline.getCurrentMarkerPosition();
        switch(position){
            case LEFT:
                drive.followTrajectorySequence(plannedTrajectories.blueCloseMarkerLeftOutakeSequence);
                drive.followTrajectorySequence(plannedTrajectories.blueCloseDropLeftSequence);
                break;

            case RIGHT:
                drive.followTrajectorySequence(plannedTrajectories.blueCloseMarkerRightOutakeSequence);
                drive.followTrajectorySequence(plannedTrajectories.blueCloseDropRightSequence);
                break;

            case MIDDLE:
                drive.followTrajectorySequence(plannedTrajectories.blueCloseMarkerMiddleOutakeSequence);
                drive.followTrajectorySequence(plannedTrajectories.blueCloseDropMiddleSequence);
                break;

            default:
                break;
        }


        //park near
        drive.followTrajectorySequence(plannedTrajectories.blueNearParkSequence);








    }

    @Override
    public void loop() {

    }

}