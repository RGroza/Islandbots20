package org.firstinspires.ftc.islandbots19.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.islandbots19.robot.CompetitionBot;

@Autonomous(name="SonarTest", group="Test")
public class SonarTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
//            telemetry.addData("Voltage: ", robot.wallDistanceFront.getVoltage());
            telemetry.update();
        }
    }
}
