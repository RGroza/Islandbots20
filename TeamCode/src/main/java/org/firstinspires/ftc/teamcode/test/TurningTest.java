package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mode.AutonomousCommands;
import org.firstinspires.ftc.teamcode.mode.AutonomousNew;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="TurningTest", group="Test")
public class TurningTest extends AutonomousNew {
    @Override
    public void runOpMode() {
        robot = new CompetitionBot(hardwareMap, telemetry);
        AutonomousCommands auto = new AutonomousCommands(robot);

        waitForStart();

        turnBy(.2, 90);
        sleep(1000);
        turnBy(.2, -90);
        sleep(1000);
        turnBy(.2, 180);
        sleep(1000);
        turnBy(.2, -180);
        sleep(1000);

//        turnByPID(.2, 90);
//        sleep(1000);
//        turnByPID(.2, -90);
//        sleep(1000);
//        turnByPID(.2, 180);
//        sleep(1000);
//        turnByPID(.2, -180);
//        sleep(1000);
    }
}
