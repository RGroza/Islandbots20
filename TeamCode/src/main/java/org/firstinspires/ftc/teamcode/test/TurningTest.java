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

        turnUntil(.3, 90);
        sleep(1000);
        turnUntil(.3, -90);
        sleep(1000);
        turnUntil(.3, 0);
        sleep(1000);
        turnUntil(.3, 180);
        sleep(1000);
    }
}
