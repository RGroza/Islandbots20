package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousCommands;
import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="StrafeTest2", group="Test")
public class StrafeTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);
        AutonomousCommands auto = new AutonomousCommands(robot);

        waitForStart();

        left(.3, 1, robot.getPitch(), true);
        right(.3, 1, robot.getPitch(), true);
    }
}
