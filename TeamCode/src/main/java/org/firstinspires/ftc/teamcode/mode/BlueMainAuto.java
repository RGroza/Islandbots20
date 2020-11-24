package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="BlueMainAuto", group="Autonomous")
public class BlueMainAuto extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        blueMainAuto(telemetry);
    }

}
