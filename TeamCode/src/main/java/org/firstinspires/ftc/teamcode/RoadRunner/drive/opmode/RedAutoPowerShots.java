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
@Autonomous(name="RedAutoPowerShots", group="Autonomous")
public class RedAutoPowerShots extends LinearOpMode {
    public static double START_X = -63.25;
    public static double START_Y = -20.75;
    public static double BACK_DIST = 40;
    public static double A_X = 0;
    public static double A_Y = -50;
    public static double B_X = 24;
    public static double B_Y = START_Y;
    public static double C_X = 48;
    public static double C_Y = -40;
    public static double PARK_X = 6;
    public static double PARK_Y = -36;
    public static double WOBBLE_X = -40;
    public static double WOBBLE_Y = -50;

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
        RingsOpenCV vision = new RingsOpenCV(true, hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPose = new Pose2d(START_X, START_Y, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        waitForStart();

        if (isStopRequested()) return;

        robot.SlideMotor.setPower(.75);
        robot.armRotateServo.setPosition(robot.ARM_MID);
        sleep(750);
        robot.SlideMotor.setPower(0);

        sleep(1500);
        int numRings = vision.getNumberRings();
        telemetry.addData("numRings: ", numRings);
        telemetry.update();

        Trajectory traj = drive.trajectoryBuilder(startingPose)
                .back(4.5)
                .build();
        drive.followTrajectory(traj);

        shootPowerShot(robot);

//        drive.turn(Math.toRadians(1.5));

        traj = drive.trajectoryBuilder(traj.end())
                .strafeRight(7.5)
                .build();
        drive.followTrajectory(traj);

        shootPowerShot(robot);

        traj = drive.trajectoryBuilder(traj.end())
                .strafeRight(7.5)
                .build();
        drive.followTrajectory(traj);

        shootPowerShot(robot);
        robot.LFlywheelMotor.setPower(0);
        robot.RFlywheelMotor.setPower(0);


        if (numRings == 0) {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .back(BACK_DIST)
                    .splineTo(new Vector2d(A_X, A_Y), Math.toRadians(270))
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);
            robot.armRotateServo.setPosition(robot.ARM_IN);

            traj = drive.trajectoryBuilder(traj.end())
                    .forward(12)
                    .build();
            drive.followTrajectory(traj);

        } else if (numRings == 1) {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .back(BACK_DIST)
                    .splineTo(new Vector2d(B_X, B_Y), Math.toRadians(270))
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);
            robot.armRotateServo.setPosition(robot.ARM_IN);

            traj = drive.trajectoryBuilder(traj.end())
                    .lineToSplineHeading(new Pose2d(PARK_X, PARK_Y, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(traj);

        } else {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .back(BACK_DIST)
                    .splineTo(new Vector2d(C_X, C_Y), Math.toRadians(270))
                    .build();

            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);
            robot.armRotateServo.setPosition(robot.ARM_IN);

/*
            traj = drive.trajectoryBuilder(traj.end())
                    .splineTo(new Vector2d(PARK_X, PARK_Y), Math.toRadians(180))
                    .build();
            drive.followTrajectory(traj);
*/

        }

        traj = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(WOBBLE_X, WOBBLE_Y), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj);

        robot.grabberServo.setPosition(robot.GRABBER_CLOSED);
    }

    public void shootPowerShot(CompetitionBot robot) {
        robot.LFlywheelMotor.setPower(.855);
        robot.RFlywheelMotor.setPower(.855);
        sleep(1500);
        robot.ringFeedServo.setPosition(robot.FEED_OPEN);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
        sleep(500);
    }

    public void shootHighGoal(CompetitionBot robot) {
        robot.LFlywheelMotor.setPower(.9);
        robot.RFlywheelMotor.setPower(.9);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_OPEN);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
    }
}