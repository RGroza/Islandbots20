package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RampingTest", group="Test")
public class RampingTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        double currentAngle = robot.getPitch();

        waitForStart();
        forward(.5, 7.5, true, false, telemetry);
        sleep(1000);
        backward(.5, 7.5, true, false, telemetry);
        sleep(1000);
        forward(.75, 7.5, true, false, telemetry);
        turnUntil(.5, currentAngle);
        sleep(1000);
        backward(.75, 7.5, true, false, telemetry);
        turnUntil(.5, currentAngle);
        sleep(1000);
    }
}
