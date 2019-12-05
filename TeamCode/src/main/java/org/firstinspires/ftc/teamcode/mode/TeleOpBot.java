package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.GamepadButton;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="TeleOpBot", group="Competition")
public class TeleOpBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
        // Gamepad 1
        GamepadButton slowToggleButton = new GamepadButton(300, false);
        GamepadButton slideUpButton = new GamepadButton(300, false);
        GamepadButton slideDownButton = new GamepadButton(300, false);
        GamepadButton slideUpLevelButton = new GamepadButton(300, false);
        GamepadButton slideDownLevelButton = new GamepadButton(300, false);
        GamepadButton grabberServoButton = new GamepadButton(300, false);
        GamepadButton armRotateButton = new GamepadButton(300, false);
        GamepadButton intakeButton = new GamepadButton(300, false);
        GamepadButton foundationServoButton = new GamepadButton(300, false);

        int slideMotorSteps = 0;
        int slideLevel = 0;

        waitForStart();
        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            boolean slowToggleBool = gamepad1.right_stick_button;

            // Gamepad 2
            double slide_y = -gamepad2.left_stick_y;

            boolean slideUpBool = gamepad2.y;
            boolean slideDownBool = gamepad2.a;
            boolean slideUpLevelBool = gamepad2.dpad_up;
            boolean slideDownLevelBool = gamepad2.dpad_down;

            boolean grabberServoBool = gamepad2.right_bumper;
            boolean armRotateServoBool = gamepad2.left_bumper;

            boolean intakeBool = gamepad2.x;
            boolean reverseIntakeBool = gamepad2.b;

            boolean foundationServoBool = gamepad2.dpad_left;

            // BUTTON DEBOUNCE
            slowToggleButton.checkStatus(slowToggleBool);
            slideUpButton.checkStatus(slideUpBool);
            slideDownButton.checkStatus(slideDownBool);
            slideUpLevelButton.checkStatus(slideUpLevelBool);
            slideDownLevelButton.checkStatus(slideDownLevelBool);
            grabberServoButton.checkStatus(grabberServoBool);
            armRotateButton.checkStatus(armRotateServoBool);
            intakeButton.checkStatus(intakeBool);
            foundationServoButton.checkStatus(foundationServoBool);

            if (slowToggleButton.pressed) {
                x /= 2;
                y /= 2;
            }

            if (slideUpButton.buttonStatus) {
                robot.SlideMotor.setPower(.75);
            } else if (slideDownButton.buttonStatus) {
                robot.SlideMotor.setPower(-.75);
            } else {
                robot.SlideMotor.setPower(0);
            }

            // Manual control of the linear slide
            robot.SlideMotor.setPower(slide_y*slide_y);

            // TODO: to be tested
            if (slideUpLevelButton.justPressed && slideLevel <= 4) {
                slideMotorSteps = 500;
                slideLevel++;
            } else if (slideDownLevelButton.justPressed && slideLevel > 0) {
                slideMotorSteps = -500;
                slideLevel--;
            }
            if (slideMotorSteps != 0) {
                double initialPos = robot.SlideMotor.getCurrentPosition();
                while (robot.SlideMotor.getCurrentPosition() - initialPos < 1) {
                    double power = slideMotorSteps > 0 ? .75 : -.75;
                    robot.SlideMotor.setPower(power);
                }
            }

            if (grabberServoButton.pressed) {
                robot.grabberServo.setPosition(.75);
            } else {
                robot.grabberServo.setPosition(.25);
            }

            if (armRotateButton.pressed) {
                robot.armRotateServo.setPosition(.75);
            } else {
                robot.armRotateServo.setPosition(.25);
            }

            if (intakeButton.pressed) {
                robot.IntakeMotor.setPower(.75);
            } else {
                robot.IntakeMotor.setPower(0);
            }

            if (foundationServoButton.pressed) {
                robot.Lfoundation.setPosition(.75);
                robot.Rfoundation.setPosition(.75);
            } else {
                robot.Lfoundation.setPosition(.25);
                robot.Rfoundation.setPosition(.25);
            }

            // MOVEMENT
            double[] powerList = robot.mecanumMove(x, y, rotation, slowToggleButton.pressed, telemetry);

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
            telemetry.update();

        }

    }
}
