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

    public boolean waitAndContinue(long initTime, long duration) {
        return (System.currentTimeMillis() - initTime > duration);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        // BUTTON DECLARE
        // Gamepad 1
        GamepadButton slowToggleButton = new GamepadButton(300, false);
        GamepadButton fastHoldButton = new GamepadButton(300, false);
        GamepadButton reverseToggleButton = new GamepadButton(300, false);

        GamepadButton intakeButton = new GamepadButton(300, false);
        GamepadButton reverseIntakeButton = new GamepadButton(300, false);
        GamepadButton flywheelButton = new GamepadButton(300, false);
        GamepadButton ringFeedButton = new GamepadButton(300, false);
        GamepadButton grabberButton = new GamepadButton(300, false);
        GamepadButton armRotateButton = new GamepadButton(300, false);
        GamepadButton increaseFlywheelButton = new GamepadButton(300, false);
        GamepadButton decreaseFlywheelButton = new GamepadButton(300, false);
        GamepadButton increaseIntakeButton = new GamepadButton(300, false);
        GamepadButton decreaseIntakeButton = new GamepadButton(300, false);
        GamepadButton slideHomeButton = new GamepadButton(300, false);
        GamepadButton slideUpButton = new GamepadButton(300, false);
        GamepadButton slideDownButton = new GamepadButton(300, false);

        double[] powerList = {0, 0, 0, 0};
        double intakePower = .75;
        double flywheelPower = .5;

        waitForStart();
        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            boolean slowToggleBool = gamepad1.right_stick_button;
            boolean fastHoldBool = gamepad1.right_bumper;
            boolean reverseToggleBool = gamepad1.left_stick_button;

            // Gamepad 2
            boolean intakeBool = gamepad1.x;
            boolean reverseIntakeBool = gamepad1.b;
            boolean flywheelBool = gamepad1.y;

            boolean ringFeedBool = gamepad1.a;
            boolean grabberBool = gamepad1.left_bumper;
            boolean armRotateBool = gamepad1.right_bumper;

            boolean increaseFlywheelBool = gamepad1.dpad_up;
            boolean decreaseFlywheelBool = gamepad1.dpad_down;
            boolean increaseIntakeBool = gamepad1.dpad_right;
            boolean decreaseIntakeBool = gamepad1.dpad_left;

            double slide_y = gamepad2.left_stick_y;
            boolean slideHome = gamepad2.a;

            boolean slideUpBool = gamepad2.dpad_up;
            boolean slideDownBool = gamepad2.dpad_down;

            // BUTTON DEBOUNCE
            // Gamepad 1
            slowToggleButton.checkStatus(slowToggleBool);
            fastHoldButton.checkStatus(fastHoldBool);
            reverseToggleButton.checkStatus(reverseToggleBool);

            // Gamepad 2
            ringFeedButton.checkStatus(ringFeedBool);
            grabberButton.checkStatus(grabberBool);
            armRotateButton.checkStatus(armRotateBool);
            increaseFlywheelButton.checkStatus(increaseFlywheelBool);
            decreaseFlywheelButton.checkStatus(decreaseFlywheelBool);
            increaseIntakeButton.checkStatus(increaseIntakeBool);
            decreaseIntakeButton.checkStatus(decreaseIntakeBool);
            intakeButton.checkStatus(intakeBool);
            reverseIntakeButton.checkStatus(reverseIntakeBool);
            flywheelButton.checkStatus(flywheelBool);
            slideHomeButton.checkStatus(slideHome);
            slideUpButton.checkStatus(slideUpBool);
            slideDownButton.checkStatus(slideDownBool);

            boolean slideActive = false;

            long initTime = 0;
            boolean waitForArm = false;

            // Adjust Intake Power
            if (intakePower > 0 && intakePower < 1) {
                if (increaseIntakeButton.justPressed) {
                    intakePower += .05;
                }
                if (decreaseIntakeButton.justPressed) {
                    intakePower -= .05;
                }
            }

            // Adjust Flywheel Power
            if (flywheelPower > 0 && flywheelPower < 1) {
                if (increaseFlywheelButton.justPressed) {
                    flywheelPower += .05;
                }
                if (decreaseFlywheelButton.justPressed) {
                    flywheelPower -= .05;
                }
            }

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

            if (flywheelButton.pressed) {
                robot.IntakeMotor.setPower(flywheelPower);
            } else {
                robot.IntakeMotor.setPower(0);
            }

            if (grabberButton.pressed) {
                robot.grabberServo.setPosition(robot.GRABBER_OPEN);
            } else {
                robot.grabberServo.setPosition(robot.GRABBER_CLOSED);
            }

            if (armRotateButton.pressed) {
                robot.grabberServo.setPosition(robot.ARM_OUT);
            } else {
                robot.grabberServo.setPosition(robot.ARM_IN);
            }

            // Linear slide
            if (slide_y > .05) {
                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(slide_y * slide_y);
                slideActive = true;
            } else if (slide_y < -.05) {
                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(-(slide_y * slide_y));
                slideActive = true;
            } else if (slideUpButton.buttonStatus) {
                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(-.75);
                slideActive = true;
            } else if (slideDownButton.buttonStatus) {
                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(.75);
                slideActive = true;
            } else {
                // no active control
                if (slideActive) {
                    // slide was active in previous loop - so we just released controls
                    robot.SlideMotor.setTargetPosition(robot.SlideMotor.getCurrentPosition() + 10);
                    robot.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.SlideMotor.setPower(.5);
                    slideActive = false;
                }
            }

            if (slideHomeButton.pressed) {
                if (robot.armRotateServo.getPosition() == CompetitionBot.ARM_OUT) {
                    robot.armRotateServo.setPosition(CompetitionBot.ARM_IN);
                    armRotateButton.pressedSwitchStatus();

                    initTime = System.currentTimeMillis();
                    waitForArm = false;
                    if (robot.SlideMotor.getCurrentPosition() > -1500) waitForArm = true;
                }
                if (!waitForArm || waitAndContinue(initTime, 500)) {
                    robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (robot.SlideMotor.getCurrentPosition() < -100) {
                        robot.SlideMotor.setPower(.75);
                    }
                    else {
                        robot.SlideMotor.setPower(0);
                        slideHomeButton.pressedSwitchStatus();
                    }
                }
            }

            // MOVEMENT
            rotation = Math.abs(rotation) < .1 ? 0 : rotation; // "Dead-zone" for joystick
            powerList = robot.mecanumMove(x, y, rotation, slowToggleButton.pressed, fastHoldButton.buttonStatus, telemetry);


            telemetry.addData("flywheelPower: ", flywheelPower);
            telemetry.addData("intakePower: ", intakePower);
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
