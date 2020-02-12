package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="DepositTest", group="Test")
public class DepositTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        robot.Lfoundation.setPosition(CompetitionBot.FOUND_RIGHT);
        robot.Rfoundation.setPosition(CompetitionBot.FOUND_LEFT);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);

        waitForStart();
        depositBlock(telemetry);
    }
}
