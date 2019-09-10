package org.firstinspires.ftc.islandbots19.mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.islandbots19.robot.CompetitionBot;
import org.firstinspires.ftc.islandbots19.robot.GamepadButton;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by KaDon on 8/26/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpBot", group="Competition")
public class TeleOpBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);
        // Gamepad 1
        GamepadButton slowToggleButton = new GamepadButton(300, false);

        /*
        GamepadButton reverseToggleButton = new GamepadButton(300, false);
        GamepadButton winchLiftUpButton = new GamepadButton(300, false);
        GamepadButton winchLiftDownButton = new GamepadButton(300, false);
        GamepadButton mineralLiftUpButton = new GamepadButton(300, false);
        GamepadButton mineralLiftDownButton = new GamepadButton(300, false);

        // Gamepad 2
        GamepadButton sweeperSlideOutButton = new GamepadButton(300, false);
        GamepadButton sweeperSlideInButton = new GamepadButton(300, false);
        GamepadButton sweeperToggleButton = new GamepadButton(300, false);
        GamepadButton dropPosToggleButton = new GamepadButton(300, false);
//        GamepadButton sweeperHalfLiftToggleButton = new GamepadButton(300, false);
        GamepadButton sweeperReverseButton = new GamepadButton(300, false);
//        GamepadButton sweeperDownToggleButton = new GamepadButton(300, false);
        GamepadButton boxPivotDropToggleButton = new GamepadButton(300, false);
        GamepadButton sweeperArmUpButton = new GamepadButton(300, false);
        GamepadButton sweeperArmDownButton = new GamepadButton(300, false);

        final double HANG_MOTOR_SPEED = 0.5;
        double sweeperSlidePos = .5;
        int sweeperArmBrakePosition = 0;
        */

        waitForStart();

        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            boolean slowToggleBool = gamepad1.y;

            /*
            boolean winchLiftUp = gamepad1.dpad_up;
            boolean winchLiftDown = gamepad1.dpad_down;
            boolean mineralLiftUp = gamepad1.left_bumper;
            boolean mineralLiftDown = gamepad1.right_bumper;

            // Gamepad 2
            boolean sweeperSlideOut = gamepad2.dpad_right;
            boolean sweeperSlideIn = gamepad2.dpad_left;
            boolean sweeperToggle = gamepad2.a;
//            boolean sweeperLiftToggle = gamepad2.x;
            boolean dropPosToggle = gamepad2.x;
//            boolean sweeperHalfLiftToggle = gamepad2.y;
//            boolean sweeperDownToggle = gamepad2.b;
            boolean sweeperReverse = gamepad2.right_bumper;
            boolean boxPivotDropToggle = gamepad2.left_bumper;
            boolean sweeperArmUp = gamepad2.dpad_up;
            boolean sweeperArmDown = gamepad2.dpad_down;

            // BUTTON DE-BOUNCE
            // Gamepad 1
            */

            slowToggleButton.checkStatus(slowToggleBool);

            /*
            winchLiftUpButton.checkStatus(winchLiftUp);
            winchLiftDownButton.checkStatus(winchLiftDown);
            mineralLiftDownButton.checkStatus(mineralLiftDown);
            mineralLiftUpButton.checkStatus(mineralLiftUp);

            // Gamepad 2
            sweeperSlideOutButton.checkStatus(sweeperSlideOut);
            sweeperSlideInButton.checkStatus(sweeperSlideIn);
            sweeperToggleButton.checkStatus(sweeperToggle);
            sweeperReverseButton.checkStatus(sweeperReverse);
            dropPosToggleButton.checkStatus(dropPosToggle);
//            sweeperHalfLiftToggleButton.checkStatus(sweeperHalfLiftToggle);
//            sweeperDownToggleButton.checkStatus(sweeperDownToggle);
            boxPivotDropToggleButton.checkStatus(boxPivotDropToggle);
            sweeperArmUpButton.checkStatus(sweeperArmUp);
            sweeperArmDownButton.checkStatus(sweeperArmDown);

            if(reverseToggleButton.pressed) {
                x = gamepad1.left_stick_x;
                y = gamepad1.left_stick_y;
            }
            */

            if(slowToggleButton.pressed) {
                x /= 2;
                y /= 2;
            }

            /*
            robot.Lights.setPower(.2);

            // SweeperMotor CRServo
            if(sweeperToggleButton.pressed) {
                robot.SweeperMotor.setPower(1);
            } else {
                robot.SweeperMotor.setPower(0); // .464
            }
            if(sweeperReverseButton.buttonStatus) {
                robot.SweeperMotor.setPower(-1);
            }
//            robot.SweeperMotor.setPower((-.5 * gamepad1.right_stick_y) + .5);



            if(sweeperHalfLiftToggleButton.justPressed) {
                robot.SweeperBoxLift.setPosition(CompetitionBot.SWEEPER_HALF_POS);
            } else if(dropPosToggleButton.justPressed) {
                robot.SweeperBoxLift.setPosition(CompetitionBot.SWEEPER_UP_POS);
            } else if(sweeperDownToggleButton.justPressed){
                robot.SweeperBoxLift.setPosition(CompetitionBot.SWEEPER_DOWN_POS);
            }


            // Sweeper Stabilize
            if(robot.SweeperArm.getCurrentPosition() > 1400) {
                if(!dropPosToggleButton.buttonStatus && !boxPivotDropToggleButton.buttonStatus) robot.SweeperBoxPivot.setPosition(.25);
            } else {
                if(!dropPosToggleButton.buttonStatus && !boxPivotDropToggleButton.buttonStatus) robot.SweeperBoxPivot.setPosition(.53);
            }
            if(boxPivotDropToggleButton.buttonStatus) {
                robot.SweeperBoxPivot.setPosition(.55);
            }

            if(dropPosToggleButton.buttonStatus) {
                sweeperArmBrakePosition = robot.SweeperArm.getCurrentPosition();
                robot.SweeperSlide.setPosition(.75);
                if(robot.SweeperArm.getCurrentPosition() < 2380) {
                    if (robot.SweeperArm.getCurrentPosition() > 1350) robot.SweeperArm.setPower(.2);
                    else if (robot.SweeperArm.getCurrentPosition() < 1300)
                        robot.SweeperArm.setPower(.6);
                    else if (robot.SweeperArm.getCurrentPosition() > 1800)
                        robot.SweeperArm.setPower(.4);
                    robot.SweeperBoxPivot.setPosition(.15);
                } else {
                    robot.SweeperArm.setPower(0);
                }

            }


            // Sweeper Slide Servo
            if(sweeperSlideOut) {
                if(sweeperSlidePos < 1) sweeperSlidePos += .01;
                robot.SweeperSlide.setPosition(sweeperSlidePos);
            } else if (sweeperSlideIn) {
                if(sweeperSlidePos > .5) sweeperSlidePos -= .01;
                robot.SweeperSlide.setPosition(sweeperSlidePos);
            }

            // Winch Lift
            if(winchLiftUpButton.buttonStatus) {
                robot.winchLift.setPower(1);
            } else if(winchLiftDownButton.buttonStatus) {
                robot.winchLift.setPower(-1);
//                robot.mineralLift.setPower(.4);
            } else {
                robot.winchLift.setPower(0);
                if(mineralLiftUpButton.buttonStatus) {
                    robot.mineralLift.setPower(1);
                } else if (mineralLiftDownButton.buttonStatus) {
                    robot.mineralLift.setPower(-1);
                } else {
                    robot.mineralLift.setPower(0);
                }
            }

            if(sweeperArmUpButton.buttonStatus) {
                if(robot.SweeperArm.getCurrentPosition() > 1300) robot.SweeperArm.setPower(.2);
                else robot.SweeperArm.setPower(.5);
                sweeperArmBrakePosition = 0;
            } else if (sweeperArmDownButton.buttonStatus) {
                if(robot.SweeperArm.getCurrentPosition() < 1300) robot.SweeperArm.setPower(-.1);
                else robot.SweeperArm.setPower(-.4);
                sweeperArmBrakePosition = 0;
            } else {
                if(sweeperArmBrakePosition == 0) {
                    sweeperArmBrakePosition = robot.SweeperArm.getCurrentPosition();
                }
                if(!dropPosToggleButton.buttonStatus) robot.SweeperArm.setPower((sweeperArmBrakePosition - robot.SweeperArm.getCurrentPosition()) * .01);
            }
            */

            // MOVEMENT
            robot.mecanumMove(x, y, rotation, slowToggleButton.pressed);

            //telemetry.addData("Arm: ", robot.SweeperArm.getCurrentPosition());
            //telemetry.addData("Slide: ", sweeperSlidePos);
            telemetry.addData("LF", robot.LFmotor.getCurrentPosition());
            telemetry.addData("LB", robot.LBmotor.getCurrentPosition());
            telemetry.addData("RF", robot.RFmotor.getCurrentPosition());
            telemetry.addData("RB", robot.RBmotor.getCurrentPosition());
            telemetry.addData("joyX: ", gamepad1.left_stick_x);
            telemetry.addData("joyY: ", gamepad1.left_stick_y);
            telemetry.addData("X: ", slowToggleButton.pressed);
            Orientation angOrientation = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Orientation", angOrientation.firstAngle);
            telemetry.update();

        }

    }
}
