package org.firstinspires.ftc.islandbots19.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.islandbots19.mode.AutonomousNew;
import org.firstinspires.ftc.islandbots19.robot.CompetitionBot;
import org.firstinspires.ftc.islandbots19.robot.TestBotInit;

/**
 * Created by KaDon on 9/23/2018.
 *
 */
// took org/firstinspires/ftc/robotcontroller/external/samples/PushbotAutoDriveToLine_Linear.java as a prototype
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousLineDetector", group="Test")
public class AutonomousLineDetector extends AutonomousNew {
    static private final double  APPROACH_SPEED  = -.1;
    static private final double  NEXT_SPEED  = .5;
    static private final double COLOR_RANGE = 100;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);
        waitForStart();
        if(opModeIsActive()) {
            runTestLineDetect(telemetry);
        }
//        telemetry.addData("Reading after check: ", 0); //"right and left in the range: ", 3);
//        telemetry.addData("Left  Level: ",  robot.LcolorSensor.blue()); //83
//        telemetry.addData("Right  Level: ",  robot.RcolorSensor.blue()); //74
//        telemetry.addData("Left Alpha  Level: ",  robot.LcolorSensor.alpha()); //304
//        telemetry.addData("Right Alpha  Level: ",  robot.RcolorSensor.alpha()); //275
//        telemetry.update();

    }
}
