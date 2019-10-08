package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BasicMotionTest", group="Competition")
public class BasicMotionTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);
        forward(0.5, 50);
        turnUntil(0.8,90);
        forward(0.5, 50);
    }
}
