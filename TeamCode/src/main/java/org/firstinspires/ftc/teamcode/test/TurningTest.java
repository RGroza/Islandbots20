package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="TurningTest", group="Test")
public class TurningTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        turnUntil(.5, 90);
        sleep(500);
        turnUntil(.5, -90);
        sleep(500);
        turnUntil(.5, 45);
        sleep(500);
        turnUntil(.5, 0);
    }
}
