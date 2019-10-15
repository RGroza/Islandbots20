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

        waitForStart();

        while(opModeIsActive()) {
            // CONTROLS
            // Gamepad 1
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            boolean slowToggleBool = gamepad1.y;

            // BUTTON DEBOUNCE
            slowToggleButton.checkStatus(slowToggleBool);

            if(slowToggleButton.pressed) {
                x /= 2;
                y /= 2;
            }


            // MOVEMENT
            robot.mecanumMove(x, y, rotation, slowToggleButton.pressed);

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
