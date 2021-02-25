package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class CustomSplineTest extends LinearOpMode {
    public static double X_I = -60;
    public static double Y_I = -24;
    public static double X_F = 0;
    public static double Y_F = -48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(X_I, Y_I, 0);
        drive.setPoseEstimate(startingPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            Trajectory traj = drive.trajectoryBuilder(startingPose)
                    .splineTo(new Vector2d(X_F, Y_F), 0)
                    .build();

            drive.followTrajectory(traj);

            sleep(2000);

            drive.followTrajectory(
                    drive.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(startingPose.getX(), startingPose.getY()), Math.toRadians(180))
                            .build()
            );

            sleep(2000);
        }
    }
}
