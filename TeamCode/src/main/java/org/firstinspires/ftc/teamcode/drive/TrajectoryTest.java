package org.firstinspires.ftc.teamcode.drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class TrajectoryTest extends LinearOpMode {
    public Vector2d transformVector(Vector2d vector){
        return new Vector2d(vector.getY(), -vector.getX());
    }
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueCloseStartPose = new Pose2d(0,0,Math.toRadians(0));

        Trajectory traj1 = drive.trajectoryBuilder(blueCloseStartPose)
                .splineTo(transformVector(new Vector2d(50, 0)), Math.toRadians(0))
                .splineTo(transformVector(new Vector2d(-20, 10)), Math.toRadians(0))
                .splineTo(transformVector(new Vector2d(0,0)), Math.toRadians(30))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
    }
}