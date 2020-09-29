package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.mode.AutonomousCommands;

@Autonomous(name="RoadRunnerTest", group="Autonomous")
public class RoadRunnerTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
//        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
//        AutonomousCommands auto = new AutonomousCommands(robot);

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Moving!");
        telemetry.update();

        // Initiate trajectory
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeRight(30).build());

        sleep(500);

        // Build trajectory to strafe left
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d()).strafeLeft(30).build());

// TODO: Debug non-functioning turning
//        sleep(2000);
//        drive.turn(Math.toRadians(90));
//        sleep(2000);
//        drive.turn(Math.toRadians(-90));

        sleep(500);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(30, 50))
                .build();

        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end())
                .strafeTo(new Vector2d(0, 50))
                .build();

        drive.followTrajectory(traj);

        sleep(500);

        // Build trajectory to follow spline vector (0, 0)
        traj = drive.trajectoryBuilder(traj.end(), true)
                .strafeTo(new Vector2d(0, 0))
                .build();

        drive.followTrajectory(traj);
    }
}
