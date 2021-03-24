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
public class TeleOpBot extends LinearOpMode {

    public boolean waitAndContinue(long initTime, long duration) {
        return (System.currentTimeMillis() - initTime > duration);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        robot.RFmotor.setDirection(DcMotor.Direction.REVERSE);
        robot.RBmotor.setDirection(DcMotor.Direction.REVERSE);
        robot.armRotateServo.setPosition(robot.ARM_MID);

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
        GamepadButton armMidButton = new GamepadButton(300, false);
        GamepadButton increaseFlywheelButton = new GamepadButton(300, false);
        GamepadButton decreaseFlywheelButton = new GamepadButton(300, false);
        GamepadButton increaseIntakeButton = new GamepadButton(300, false);
        GamepadButton decreaseIntakeButton = new GamepadButton(300, false);
        GamepadButton slideHomeButton = new GamepadButton(300, false);
        GamepadButton slideUpButton = new GamepadButton(300, false);
        GamepadButton slideDownButton = new GamepadButton(300, false);

        double[] powerList = {0, 0, 0, 0};
        double intakePower = .75;
        double flywheelPower = .65;
        int initFlywheelPos = 0;
        int flywheelEncoderSpeed = 0;

        long initRingFeedTime = 0;
        boolean ringFeedDelaying = false;
        long initFlywheelTime = 0;
        boolean flywheelMeasuring = false;

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
            boolean intakeBool = gamepad2.x;
            boolean reverseIntakeBool = gamepad2.b;
            boolean flywheelBool = gamepad2.y;

            boolean ringFeedBool = gamepad2.right_bumper;
            boolean grabberBool = gamepad2.left_stick_button;
            boolean armRotateBool = gamepad2.right_stick_button;
            boolean armMidBool = gamepad2.left_bumper;

            boolean increaseFlywheelBool = gamepad2.dpad_up;
            boolean decreaseFlywheelBool = gamepad2.dpad_down;
            boolean increaseIntakeBool = gamepad2.dpad_right;
            boolean decreaseIntakeBool = gamepad2.dpad_left;

            double slide_y = gamepad2.left_stick_y;
            boolean slideHome = gamepad2.a;

//            boolean slideUpBool = gamepad2.dpad_up;
//            boolean slideDownBool = gamepad2.dpad_down;

            // BUTTON DEBOUNCE
            // Gamepad 1
            slowToggleButton.checkStatus(slowToggleBool);
            fastHoldButton.checkStatus(fastHoldBool);
            reverseToggleButton.checkStatus(reverseToggleBool);

            // Gamepad 2
            ringFeedButton.checkStatus(ringFeedBool);
            grabberButton.checkStatus(grabberBool);
            armRotateButton.checkStatus(armRotateBool);
            armMidButton.checkStatus(armMidBool);
            increaseFlywheelButton.checkStatus(increaseFlywheelBool);
            decreaseFlywheelButton.checkStatus(decreaseFlywheelBool);
            increaseIntakeButton.checkStatus(increaseIntakeBool);
            decreaseIntakeButton.checkStatus(decreaseIntakeBool);
            intakeButton.checkStatus(intakeBool);
            reverseIntakeButton.checkStatus(reverseIntakeBool);
            flywheelButton.checkStatus(flywheelBool);
            slideHomeButton.checkStatus(slideHome);
//            slideUpButton.checkStatus(slideUpBool);
//            slideDownButton.checkStatus(slideDownBool);

            boolean slideActive = false;


            // Adjust Intake Power
            if (intakePower > -.025 && intakePower < 1.025) {
                if (intakePower < 1 && increaseIntakeButton.justPressed) {
                    intakePower += .05;
                }
                if (intakePower > 0 && decreaseIntakeButton.justPressed) {
                    intakePower -= .05;
                }
            }

            // Adjust Flywheel Power
            if (flywheelPower > -.01 && flywheelPower < 1.01) {
                if (flywheelPower < 1 && increaseFlywheelButton.justPressed) {
                    flywheelPower += .025;
                }
                if (flywheelPower > 0 && decreaseFlywheelButton.justPressed) {
                    flywheelPower -= .025;
                }
            }

            if (reverseToggleButton.pressed) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
            }

            if (reverseIntakeButton.buttonStatus) {
                robot.IntakeMotor.setPower(-intakePower);
            } else {
                if (intakeButton.pressed) {
                    robot.IntakeMotor.setPower(intakePower);
                } else {
                    robot.IntakeMotor.setPower(0);
                }
            }

            if (flywheelButton.pressed) {
                robot.LFlywheelMotor.setPower(flywheelPower);
                robot.RFlywheelMotor.setPower(flywheelPower - .1);
            } else {
                robot.LFlywheelMotor.setPower(0);
                robot.RFlywheelMotor.setPower(0);
            }

            if (grabberButton.pressed) {
                robot.grabberServo.setPosition(robot.GRABBER_OPEN);
                telemetry.addLine("grabber: OPEN");
            } else {
                robot.grabberServo.setPosition(robot.GRABBER_CLOSED);
                telemetry.addLine("grabber: CLOSED");
            }

            if (armRotateButton.pressed) {
                robot.armRotateServo.setPosition(robot.ARM_OUT);
                telemetry.addLine("arm: OUT");
            } else if (armMidButton.pressed) {
                robot.armRotateServo.setPosition(robot.ARM_MID);
                telemetry.addLine("arm: MID");
            } else {
                robot.armRotateServo.setPosition(robot.ARM_IN);
                telemetry.addLine("arm: IN");
            }

/*
            if (ringFeedButton.pressed) {
                robot.ringFeedServo.setPosition(robot.FEED_OPEN);
                telemetry.addLine("feed: OPEN");
            } else {
                robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
                telemetry.addLine("feed: CLOSED");
            }
*/

            if (ringFeedButton.buttonStatus) {
                robot.ringFeedServo.setPosition(robot.FEED_OPEN);
                telemetry.addLine("feed: OPEN");
                initRingFeedTime = System.currentTimeMillis();
            }
            if (!ringFeedButton.buttonStatus && waitAndContinue(initRingFeedTime, 500)) {
                robot.ringFeedServo.setPosition(robot.FEED_CLOSED);
                telemetry.addLine("feed: CLOSED");
            }


            // Linear slide
            if (slide_y > .05) {
//                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(slide_y * slide_y);
                slideActive = true;
            } else if (slide_y < -.05) {
//                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(-(slide_y * slide_y));
                slideActive = true;
            } else if (slideUpButton.buttonStatus) {
//                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(-.75);
                slideActive = true;
            } else if (slideDownButton.buttonStatus) {
//                robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.SlideMotor.setPower(.75);
                slideActive = true;
            } else {
                robot.SlideMotor.setPower(0);
            }
/*
            else {
                // no active control
                if (slideActive) {
                    // slide was active in previous loop - so we just released controls
                    robot.SlideMotor.setTargetPosition(robot.SlideMotor.getCurrentPosition() + 10);
                    robot.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.SlideMotor.setPower(.5);
                    slideActive = false;
                }
            }
*/

/*
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
*/

            // Flywheel Encoder Speed
            if (flywheelMeasuring && waitAndContinue(initFlywheelTime, 500)) {
                flywheelEncoderSpeed = 2*(robot.LFlywheelMotor.getCurrentPosition() - initFlywheelPos);
                flywheelMeasuring = false;
            }
            if (!flywheelMeasuring) {
                initFlywheelTime = System.currentTimeMillis();
                initFlywheelPos = robot.LFlywheelMotor.getCurrentPosition();
                flywheelMeasuring = true;
            }

            // MOVEMENT
            rotation = Math.abs(rotation) < .1 ? 0 : -rotation; // "Dead-zone" for joystick
            powerList = robot.mecanumMove(x, y, rotation, slowToggleButton.pressed, fastHoldButton.buttonStatus, telemetry);


            telemetry.addData("flywheelPower: ", flywheelPower);
            telemetry.addData("flywheelSpeed: ", flywheelEncoderSpeed);
            telemetry.addData("LeftFlywheelPos", robot.LFlywheelMotor.getCurrentPosition());
            telemetry.addData("RightFlywheelPos", robot.RFlywheelMotor.getCurrentPosition());
            telemetry.addData("intakePower: ", intakePower);
//            telemetry.addData("LF Pos: ", robot.LFmotor.getCurrentPosition());
//            telemetry.addData("LF Pow: ", Math.round(powerList[0] * 100.0) / 100.0);
//            telemetry.addData("LB Pos: ", robot.LBmotor.getCurrentPosition());
//            telemetry.addData("LB Pow: ", Math.round(powerList[1] * 100.0) / 100.0);
//            telemetry.addData("RF Pos: ", robot.RFmotor.getCurrentPosition());
//            telemetry.addData("RF Pow: ", Math.round(powerList[2] * 100.0) / 100.0);
//            telemetry.addData("RB Pos: ", robot.RBmotor.getCurrentPosition());
//            telemetry.addData("RB Pow: ", Math.round(powerList[3] * 100.0) / 100.0);
//            telemetry.addData("joyX: ", gamepad1.left_stick_x);
//            telemetry.addData("joyY: ", gamepad1.left_stick_y);
            telemetry.addData("slowToggle: ", slowToggleButton.pressed);
            Orientation angOrientation = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Orientation", angOrientation.firstAngle);
            telemetry.update();

        }

    }
}
