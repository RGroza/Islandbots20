package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="EncoderRampTest", group="Test")
public class EncoderRampTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();

        encoderRampTest(telemetry);
    }
}
