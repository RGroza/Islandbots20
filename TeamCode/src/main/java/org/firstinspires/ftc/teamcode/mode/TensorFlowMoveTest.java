package org.firstinspires.ftc.islandbots19.mode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.islandbots19.robot.CompetitionBot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="TensorFlowMoveTest", group="Test")
public class TensorFlowMoveTest extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();
        if(opModeIsActive()) {
            activateTfod();
        }
        while(opModeIsActive()) {
            boolean found = runTensorFlow(telemetry, false);
            telemetry.addData("Found: ", found);
            telemetry.update();
//            telemetry.addData("Left  Level: ", robot.LcolorSensor.red() + ", " + robot.LcolorSensor.green() + ", " +  robot.LcolorSensor.blue());
//            telemetry.addData("Right  Level: ", robot.RcolorSensor.red() + ", " + robot.RcolorSensor.green() + ", " + robot.RcolorSensor.blue());
//            telemetry.addData("Left Alpha  Level: ",  robot.LcolorSensor.alpha());
//            telemetry.addData("Right Alpha  Level: ",  robot.RcolorSensor.alpha());
//            telemetry.update();
        }
    }
}
