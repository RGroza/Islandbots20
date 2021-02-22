package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class TranslationPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            sleep(500);

            traj = drive.trajectoryBuilder(traj.end())
                    .strafeLeft(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            sleep(500);

            traj = drive.trajectoryBuilder(traj.end())
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            sleep(500);

            traj = drive.trajectoryBuilder(traj.end())
                    .strafeRight(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            sleep(500);

//            startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90)));
        }
    }
}
