package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="ColorTest", group="Test")
public class ColorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
//        robot.LcolorSensor.enableLed(true);
//        robot.RcolorSensor.enableLed(true);

        waitForStart();
        while(opModeIsActive()) {
            NormalizedRGBA colorsL = robot.LcolorSensor.getNormalizedColors();
            telemetry.addData("left R: ", colorsL.red);
            telemetry.addData("left G: ", colorsL.green);
            telemetry.addData("left B: ", colorsL.blue);
            telemetry.addData("left A: ", colorsL.alpha);
            NormalizedRGBA colorsR = robot.RcolorSensor.getNormalizedColors();
            telemetry.addData("right R: ", colorsR.red);
            telemetry.addData("right G: ", colorsR.green);
            telemetry.addData("right B: ", colorsR.blue);
            telemetry.addData("right A: ", colorsR.alpha);
            telemetry.update();
        }
    }
}
