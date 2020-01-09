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
        robot.LcolorSensor.enableLed(true);
        robot.RcolorSensor.enableLed(true);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("left R: ", robot.LcolorSensor.red());
            telemetry.addData("left G: ", robot.LcolorSensor.green());
            telemetry.addData("left B: ", robot.LcolorSensor.blue());
            telemetry.addData("left A: ", robot.LcolorSensor.alpha());
            telemetry.addData("right R: ", robot.RcolorSensor.red());
            telemetry.addData("right G: ", robot.RcolorSensor.green());
            telemetry.addData("right B: ", robot.RcolorSensor.blue());
            telemetry.addData("right A: ", robot.RcolorSensor.alpha());
            telemetry.update();
        }
    }
}
