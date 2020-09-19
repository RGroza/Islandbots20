package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name="RoadRunnerTest", group="Autonomous")
public class RoadRunnerTest extends LinearOpMode {
    public static double DISTANCE = 10; // in

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        // Build trajectory to strafe right
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Moving!");
        telemetry.update();

        // Initiate trajectory
        drive.followTrajectory(trajectory);

        sleep(2000);

        // Build trajectory to strafe left
        trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(DISTANCE)
                .build();

        drive.followTrajectory(trajectory);

        sleep(2000);

        // Build trajectory to follow spline vector (30, 30)
        trajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        drive.followTrajectory(trajectory);

        sleep(2000);

        // Build trajectory to follow spline vector (0, 0)
        trajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 0), 0)
                .build();

        drive.followTrajectory(trajectory);
    }
}
