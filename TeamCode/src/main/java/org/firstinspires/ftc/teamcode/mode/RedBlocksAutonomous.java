package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedBlocksAuto", group="Autonomous")
public class RedBlocksAutonomous extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        initVuforia();
        telemetry.addLine("Vuforia Initialized!");
        telemetry.update();

        waitForStart();
        runRedAutonomous(telemetry);

    }
}
