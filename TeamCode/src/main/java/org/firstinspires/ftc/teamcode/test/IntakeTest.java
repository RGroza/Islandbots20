package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="IntakeTest", group="Test")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        int initPos = robot.SlideMotor.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {
            while (Math.abs(robot.SlideMotor.getCurrentPosition() - initPos) < 100) {
                robot.SlideMotor.setPower(.05);
                telemetry.addData("initial pos: ", initPos);
                telemetry.addData("pos: ", robot.SlideMotor.getCurrentPosition());
                telemetry.update();
            }
            robot.SlideMotor.setPower(0);
        }

    }
}