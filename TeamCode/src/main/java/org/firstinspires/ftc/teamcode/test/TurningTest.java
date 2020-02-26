package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="TurningTest", group="Test")
public class TurningTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        turnByPID(.75, 90);
        sleep(500);
        turnByPID(.75, -180);
        sleep(500);
        turnByPID(.75, 45);
        sleep(500);
        turnByPID(.5, 90);
        sleep(500);
        turnByPID(.5, -180);
        sleep(500);
        turnByPID(.5, 45);
        sleep(500);
        turnByPID(.25, 90);
        sleep(500);
        turnByPID(.25, -180);
        sleep(500);
        turnByPID(.25, 45);
        sleep(500);
    }
}
