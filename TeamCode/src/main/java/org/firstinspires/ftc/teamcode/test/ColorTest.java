package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="ColorTest", group="Test")
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            robot.testColor.enableLed(true);
            telemetry.addData("R: ", robot.testColor.red());
            telemetry.addData("G: ", robot.testColor.green());
            telemetry.addData("B: ", robot.testColor.blue());
            telemetry.addData("Alpha: ", robot.testColor.alpha());
            telemetry.update();
        }
    }
}
