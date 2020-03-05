package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="CollisionTest", group="Test")
public class CollisionTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        forward(.5, 2, robot.getPitch(), true, false, telemetry);
        telemetry.addLine("Collision detected!");
        telemetry.update();
        sleep(1000);
    }
}
