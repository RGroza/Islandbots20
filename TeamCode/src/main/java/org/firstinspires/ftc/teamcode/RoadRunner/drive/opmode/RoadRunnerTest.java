package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.mode.AutonomousCommands;
import org.firstinspires.ftc.teamcode.mode.AutonomousNew;

@Autonomous(name="RoadRunnerTest", group="Autonomous")
public class RoadRunnerTest extends AutonomousNew {

    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Moving!");
        telemetry.update();

//        turnByPID(.5, 90);
//        sleep(2000);
//        turnByPID(.5, -90);
//        sleep(2000);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d()).strafeTo(new Vector2d(0, -30)).build();
        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).strafeTo(new Vector2d(30, 0)).build();
        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).strafeTo(new Vector2d(0, 0)).build();
        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).strafeTo(new Vector2d(30, -30)).build();
        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).strafeTo(new Vector2d(0, -30)).build();
        drive.followTrajectory(traj);

        sleep(500);

        traj = drive.trajectoryBuilder(traj.end()).strafeTo(new Vector2d(0, 0)).build();
        drive.followTrajectory(traj);

        sleep(1000);

        turnUntilPID(0.5, 0);
    }
}
