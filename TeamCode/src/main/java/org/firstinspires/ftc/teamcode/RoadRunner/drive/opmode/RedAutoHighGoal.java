package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.vision.RingsOpenCV;

@Config
@Autonomous(name="RedAutoHighGoal", group="Autonomous")
public class RedAutoHighGoal extends LinearOpMode {
    public static double START_X = -63.25;
    public static double START_Y = -40;
    public static double SHOOTING_X = -1.25;
    public static double SHOOTING_Y = -36;
    public static double A_X = 4;
    public static double A_Y = -46;
    public static double B_X = 30;
    public static double B_Y = -44;
    public static double C_X = 52;
    public static double C_Y = -46;
    public static double PARK_X = 12;
    public static double PARK_Y = -36;
    public static double WOBBLE_X = -40;
    public static double WOBBLE_Y = -40;

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
        RingsOpenCV vision = new RingsOpenCV(false, hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(START_X, START_Y, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        waitForStart();

        if (isStopRequested()) return;

        robot.SlideMotor.setPower(.75);
        robot.armRotateServo.setPosition(robot.ARM_AUTO);
        sleep(250);
        robot.SlideMotor.setPower(0);

        sleep(1500);
        int numRings = vision.getNumberRings();
        telemetry.addData("numRings: ", numRings);
        telemetry.update();

        Trajectory traj = drive.trajectoryBuilder(startingPose)
                .lineTo(new Vector2d(-36, -60))
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .back(6)
                .build();
        drive.followTrajectory(traj);

        traj = drive.trajectoryBuilder(traj.end())
                .lineTo(new Vector2d(SHOOTING_X, SHOOTING_Y))
                .build();
        drive.followTrajectory(traj);


        for (int i = 0; i < 3; i++) {
            shootHighGoal(robot);
        }
        robot.LFlywheelMotor.setPower(0);
        robot.RFlywheelMotor.setPower(0);


        if (numRings == 0) {
            traj = drive.trajectoryBuilder(traj.end())
                    .lineToLinearHeading(new Pose2d(A_X, A_Y, Math.toRadians(90)))
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .lineTo(new Vector2d(PARK_X, PARK_Y))
                    .build();
            drive.followTrajectory(traj);

//            traj = drive.trajectoryBuilder(traj.end())
//                    .forward(6)
//                    .build();
//            drive.followTrajectory(traj);
//
//            drive.turn(Math.toRadians(0));
//
//            traj = drive.trajectoryBuilder(traj.end(), true)
//                    .splineTo(new Vector2d(WOBBLE_X, WOBBLE_Y), Math.toRadians(90))
//                    .build();
//            drive.followTrajectory(traj);

        } else if (numRings == 1) {
            traj = drive.trajectoryBuilder(traj.end())
                    .lineTo(new Vector2d(B_X, B_Y))
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .lineTo(new Vector2d(PARK_X, PARK_Y))
                    .build();
            drive.followTrajectory(traj);

        } else {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(C_X, C_Y), Math.toRadians(270))
                    .build();

            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .splineTo(new Vector2d(PARK_X, PARK_Y), Math.toRadians(180))
                    .build();
            drive.followTrajectory(traj);

        }

        robot.SlideMotor.setPower(-.75);
        sleep(500);
        robot.SlideMotor.setPower(0);

    }

    public void shootHighGoal(CompetitionBot robot) {
        robot.LFlywheelMotor.setPower(robot.HIGH_GOAL_SPEED);
        robot.RFlywheelMotor.setPower(robot.HIGH_GOAL_SPEED - robot.FLYWHEEL_SPEED_DIFF);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_OPEN);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
    }
}