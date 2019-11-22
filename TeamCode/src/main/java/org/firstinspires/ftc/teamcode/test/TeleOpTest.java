package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.GamepadButton;
import org.firstinspires.ftc.teamcode.robot.TestBotInit;

@TeleOp(name="TeleOpTest", group="Test")
public class TeleOpTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        GamepadButton slowToggleButton = new GamepadButton(300, false);

        waitForStart();
        while(opModeIsActive()) {
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            boolean slowToggleBool = gamepad1.y;

            slowToggleButton.checkStatus(slowToggleBool);

            if (y != 0) {
                robot.setMotors(y, y, y, y);
            } else {
                robot.setMotors(rotation, rotation, -rotation, -rotation);
            }

            // robot.mecanumMove(x, y, rotation, slowToggleButton.pressed);

            telemetry.addData("A: RF:  ", robot.RFmotor.getCurrentPosition());
            telemetry.addData("B: RB:  ", robot.RBmotor.getCurrentPosition());
            telemetry.addData("X: LF:  ", robot.LFmotor.getCurrentPosition());
            telemetry.addData("Y: LB:  ", robot.LBmotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
