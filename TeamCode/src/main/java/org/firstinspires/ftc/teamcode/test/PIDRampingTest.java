package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="PIDRampingTest", group="Test")
public class PIDRampingTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        forward(.5, 7.5, true, true, telemetry);
        sleep(1000);
        backward(.5, 7.5, true, true, telemetry);
        sleep(1000);
    }
}
