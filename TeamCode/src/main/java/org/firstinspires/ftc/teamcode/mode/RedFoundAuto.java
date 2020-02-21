package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedFoundAuto", group="Autonomous")
public class RedFoundAuto extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);

//        initVuforia();
//        telemetry.addLine("Vuforia Initialized!");
//        telemetry.update();

        waitForStart();
        redFoundAuto(true, telemetry);
    }
}
