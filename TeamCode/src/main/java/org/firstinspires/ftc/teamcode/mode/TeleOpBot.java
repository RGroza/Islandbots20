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
        GamepadButton reverseIntakeButton = new GamepadButton(300, false);
        GamepadButton foundationServosButton = new GamepadButton(300, false);
        GamepadButton reverseToggleButton = new GamepadButton(300, false);

        int slideMotorSteps = 0;

        waitForStart();
        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            boolean slowToggleBool = gamepad1.right_stick_button;

            boolean foundationServosBool = gamepad1.a;

            boolean reverseToggleBool = gamepad1.left_stick_button;

            // Gamepad 2
            double slide_y = gamepad2.left_stick_y;

            boolean slideUpBool = gamepad2.y;
            boolean slideDownBool = gamepad2.a;
            boolean slideUpLevelBool = gamepad2.dpad_up;
            boolean slideDownLevelBool = gamepad2.dpad_down;

            boolean grabberServoBool = gamepad2.right_bumper;
            boolean armRotateServoBool = gamepad2.left_bumper;

            boolean intakeBool = gamepad2.x;
            boolean reverseIntakeBool = gamepad2.b;

            // BUTTON DEBOUNCE
            slowToggleButton.checkStatus(slowToggleBool);
            slideUpButton.checkStatus(slideUpBool);
            slideDownButton.checkStatus(slideDownBool);
            slideUpLevelButton.checkStatus(slideUpLevelBool);
            slideDownLevelButton.checkStatus(slideDownLevelBool);
            grabberServoButton.checkStatus(grabberServoBool);
            armRotateButton.checkStatus(armRotateServoBool);
            intakeButton.checkStatus(intakeBool);
            reverseIntakeButton.checkStatus(reverseIntakeBool);
            foundationServosButton.checkStatus(foundationServosBool);
            reverseToggleButton.checkStatus(reverseToggleBool);


            if(reverseToggleButton.pressed) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
            }
            if (slowToggleButton.pressed) {
                x *= .75;
                y *= .75;
            }

            if (slideUpButton.buttonStatus) {
                robot.SlideMotor.setPower(-.75);
            } else if (slideDownButton.buttonStatus) {
                robot.SlideMotor.setPower(.75);
            } else {
                robot.SlideMotor.setPower(0);
            }

            // Manual control of the linear slide
            if (slide_y > 0) {
                robot.SlideMotor.setPower(slide_y * slide_y);
            } else if (slide_y < 0) {
                robot.SlideMotor.setPower(-slide_y * slide_y);
            }

            double initialPos = robot.SlideMotor.getCurrentPosition();
            // TODO: to be tested and add slideLevel if necessary
            if (slideUpLevelButton.justPressed) {
                slideMotorSteps = 250;
                initialPos = robot.SlideMotor.getCurrentPosition();
            } else if (slideDownLevelButton.justPressed) {
                slideMotorSteps = -250;
                initialPos = robot.SlideMotor.getCurrentPosition();
            }
            if (slideMotorSteps != 0) {
                while (Math.abs(robot.SlideMotor.getCurrentPosition() - initialPos) < 1) {
                    double power = slideMotorSteps > 0 ? .75 : -.75;
                    robot.SlideMotor.setPower(power);
                }
            }

            if (grabberServoButton.pressed) {
                robot.grabberServo.setPosition(.65);
            } else {
                robot.grabberServo.setPosition(.35);
            }

            if (armRotateButton.pressed) {
                robot.armRotateServo.setPosition(.45);
            } else {
                robot.armRotateServo.setPosition(.08);
            }

            if (reverseIntakeButton.buttonStatus) {
                robot.IntakeMotor.setPower(-.75);
            } else {
                if (intakeButton.pressed) {
                    robot.IntakeMotor.setPower(.75);
                } else {
                    robot.IntakeMotor.setPower(0);
                }
            }

            if (foundationServosButton.pressed) {
                robot.Lfoundation.setPosition(.84);
                robot.Rfoundation.setPosition(.5);
            } else {
                robot.Lfoundation.setPosition(.57);
                robot.Rfoundation.setPosition(.8);
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
