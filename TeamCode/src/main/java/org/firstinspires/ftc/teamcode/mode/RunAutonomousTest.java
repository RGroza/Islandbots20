package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RunAutoTest", group="Test")
public class RunAutonomousTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        initVuforia();

        waitForStart();
        boolean returnedVal = detectSkyStone();
        telemetry.addData("Returned Value: ", returnedVal);
    }
}