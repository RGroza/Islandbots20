package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="PIDTurningTest", group="Test")
public class PIDTurningTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();

        turnUntilPID(.3, 90);
        sleep(1000);
        turnUntilPID(.3, -90);
        sleep(1000);
        turnUntilPID(.3, 0);
        sleep(1000);
        turnUntilPID(.3, 180);
        sleep(1000);
    }
}
