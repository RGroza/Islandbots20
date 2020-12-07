package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="SonarTest", group="Test")
public class SonarTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
//            telemetry.addData("Voltage: ", robot.sonarDistance.getVoltage());
            telemetry.update();
        }
    }
}
