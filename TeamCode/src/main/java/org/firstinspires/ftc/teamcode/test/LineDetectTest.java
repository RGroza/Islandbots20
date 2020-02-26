package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="LineDetectTest", group="Test")
public class LineDetectTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        detectLineAndStop(true, true, .3, 10, robot.getPitch(), telemetry);
    }
}
