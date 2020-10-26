package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.GamepadButton;


@TeleOp(name="TeleOpBot", group="Competition")
public class FlywheelShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        // BUTTON DECLARE
        // Gamepad 1
        GamepadButton slowToggleButton = new GamepadButton(300, false);
        GamepadButton fastHoldButton = new GamepadButton(300, false);
        GamepadButton reverseToggleButton = new GamepadButton(300, false);
        GamepadButton RingFeedButton = new GamepadButton(300, false);
        GamepadButton WobbleArmToggleButon = new GamepadButton(300, false);
        GamepadButton fullForwardButton = new GamepadButton(300, false);
        GamepadButton fullBackwardButton = new GamepadButton(300, false);
        GamepadButton fullLeftButton = new GamepadButton(300, false);
        GamepadButton fullRightButton = new GamepadButton(300, false);
        GamepadButton intakeButton = new GamepadButton(300, false);
        GamepadButton reverseIntakeButton = new GamepadButton(300, false);

        double[] powerList = {0, 0, 0, 0};

        waitForStart();
        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            boolean slowToggleBool = gamepad1.right_stick_button;
            boolean reverseToggleBool = gamepad1.left_stick_button;

            boolean RingFeedBool = gamepad1.a;
            boolean WobbleArmToggleBool = gamepad1.b;

            boolean fastHoldBool = gamepad1.right_bumper;

            boolean fullForwardBool = gamepad1.dpad_up;
            boolean fullBackwardBool = gamepad1.dpad_down;
            boolean fullLeftBool = gamepad1.dpad_left;
            boolean fullRightBool = gamepad1.dpad_right;

            // BUTTON DEBOUNCE
            // Gamepad 1
            slowToggleButton.checkStatus(slowToggleBool);
            reverseToggleButton.checkStatus(reverseToggleBool);
            RingFeedButton.checkStatus(RingFeedBool);
            WobbleArmToggleButon.checkStatus(WobbleArmToggleBool);
            fastHoldButton.checkStatus(fastHoldBool);
            fullForwardButton.checkStatus(fullForwardBool);
            fullBackwardButton.checkStatus(fullBackwardBool);
            fullLeftButton.checkStatus(fullLeftBool);
            fullRightButton.checkStatus(fullRightBool);


            if (reverseToggleButton.pressed) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
            }

            if (reverseIntakeButton.buttonStatus) {
                robot.IntakeMotor.setPower(-1);
            } else {
                if (intakeButton.pressed) {
                    robot.IntakeMotor.setPower(1);
                } else {
                    robot.IntakeMotor.setPower(0);
                }
            }

            // MOVEMENT
            if (fullForwardButton.buttonStatus) {
                robot.setMotors(1, 1, 1, 1);
            } else if (fullBackwardButton.buttonStatus) {
                robot.setMotors(-1, -1, -1, -1);
            } else if (fullLeftButton.buttonStatus) {
                robot.setMotors(-1, 1, 1, -1);
            } else if (fullRightButton.buttonStatus) {
                robot.setMotors(1, -1, -1, 1);
            } else {
                rotation = Math.abs(rotation) < .1 ? 0 : rotation; // "Dead-zone" for joystick
                powerList = robot.mecanumMove(x, y, rotation, slowToggleButton.pressed, fastHoldButton.buttonStatus, telemetry);
            }

            telemetry.addData("LF Pos: ", robot.LFmotor.getCurrentPosition());
            telemetry.addData("LF Pow: ", Math.round(powerList[0] * 100.0) / 100.0);
            telemetry.addData("LB Pos: ", robot.LBmotor.getCurrentPosition());
            telemetry.addData("LB Pow: ", Math.round(powerList[1] * 100.0) / 100.0);
            telemetry.addData("RF Pos: ", robot.RFmotor.getCurrentPosition());
            telemetry.addData("RF Pow: ", Math.round(powerList[2] * 100.0) / 100.0);
            telemetry.addData("RB Pos: ", robot.RBmotor.getCurrentPosition());
            telemetry.addData("RB Pow: ", Math.round(powerList[3] * 100.0) / 100.0);
            telemetry.addData("joyX: ", gamepad1.left_stick_x);
            telemetry.addData("joyY: ", gamepad1.left_stick_y);
            telemetry.addData("X: ", slowToggleButton.pressed);
            Orientation angOrientation = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Orientation", angOrientation.firstAngle);
            telemetry.addData("Slide Pos: ", robot.SlideMotor.getCurrentPosition());
            telemetry.addData("Sonar: ", robot.sonarDistance.getVoltage());
            telemetry.update();

        }

    }
}
