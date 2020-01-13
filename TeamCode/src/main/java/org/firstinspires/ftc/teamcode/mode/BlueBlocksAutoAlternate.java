package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueBlocksAlternate", group="Autonomous")
public class BlueBlocksAutoAlternate extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

        initVuforia();
        telemetry.addLine("Vuforia Initialized!");
        telemetry.update();

        waitForStart();
        sleep(10000);
        backward(.4, 7.5, true);
        runBlueBlocksAuto(telemetry);

    }
}
