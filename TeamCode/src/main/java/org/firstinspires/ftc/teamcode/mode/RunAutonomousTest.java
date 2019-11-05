package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RunAutoTest", group="Test")
public class RunAutonomousTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        initVuforia();

        waitForStart();
        String returnedVal = detectSkyStone(true);
        telemetry.addData("Returned Value: ", returnedVal);
        telemetry.update();
        sleep(2000);
    }
}