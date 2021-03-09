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
    public static double INIT_X = -36;
    public static double INIT_Y = -60;
    public static double A_X = -8;
    public static double A_Y = INIT_Y;
    public static double B_X = 30;
    public static double B_Y = -48;
    public static double C_X = 40;
    public static double C_Y = INIT_Y;
    public static double PARK_X = 12;
    public static double PARK_Y = -36;

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
        robot.armRotateServo.setPosition(robot.ARM_MID);
        sleep(250);
        robot.SlideMotor.setPower(0);

        sleep(1500);
        int numRings = vision.getNumberRings();
        telemetry.addData("numRings: ", numRings);
        telemetry.update();

        Trajectory traj = drive.trajectoryBuilder(startingPose)
                .back(3)
                .build();
        drive.followTrajectory(traj);

        sleep(250);

        traj = drive.trajectoryBuilder(traj.end())
                .strafeRight(4)
                .build();
        drive.followTrajectory(traj);

        robot.FlywheelMotor.setPower(.9);
        sleep(500);
        for (int i = 0; i < 3; i++) {
            shootHighGoal(robot);
        }
        robot.FlywheelMotor.setPower(0);

        traj = drive.trajectoryBuilder(traj.end(), true)
                .lineTo(new Vector2d(INIT_X, INIT_Y))
                .build();
        drive.followTrajectory(traj);


        if (numRings == 0) {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(A_X, A_Y), 0)
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .forward(6)
                    .build();
            drive.followTrajectory(traj);

        } else if (numRings == 1) {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(B_X, B_Y), Math.toRadians(45))
                    .build();
            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .lineToSplineHeading(new Pose2d(PARK_X, PARK_Y, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(traj);

            robot.armRotateServo.setPosition(robot.ARM_IN);
            robot.grabberServo.setPosition(robot.GRABBER_CLOSED);

        } else {
            traj = drive.trajectoryBuilder(traj.end(), true)
                    .splineTo(new Vector2d(C_X, C_Y), 0)
                    .build();

            drive.followTrajectory(traj);

            sleep(500);
            robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            sleep(250);

            traj = drive.trajectoryBuilder(traj.end())
                    .splineTo(new Vector2d(PARK_X, PARK_Y), Math.toRadians(180))
                    .build();
            drive.followTrajectory(traj);

            robot.armRotateServo.setPosition(robot.ARM_IN);
            robot.grabberServo.setPosition(robot.GRABBER_CLOSED);

        }

    }

    public void shootPowerShot(CompetitionBot robot) {
        robot.FlywheelMotor.setPower(.85);
        sleep(1000);
        robot.ringFeedServo.setPosition(robot.FEED_OPEN);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
        sleep(500);
    }

    public void shootHighGoal(CompetitionBot robot) {
        robot.FlywheelMotor.setPower(.9);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_OPEN);
        sleep(500);
        robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
    }
}