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
//            double left = -gamepad1.left_stick_y;
//            double right = -gamepad1.right_stick_y;
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;
            boolean slowToggleBool = gamepad1.y;

            slowToggleButton.checkStatus(slowToggleBool);

//            robot.RFmotor.setPower(right);
//            robot.RBmotor.setPower(right);
//            robot.LFmotor.setPower(left);
//            robot.LBmotor.setPower(left);

            robot.mecanumMove(x, y, rotation, slowToggleButton.pressed);

            telemetry.addData("A: RF:  ", robot.RFmotor.getCurrentPosition());
            telemetry.addData("B: RB:  ", robot.RBmotor.getCurrentPosition());
            telemetry.addData("X: LF:  ", robot.LFmotor.getCurrentPosition());
            telemetry.addData("Y: LB:  ", robot.LBmotor.getCurrentPosition());
//            telemetry.addData("RColor: ", Rcolor[0] + ", " + Rcolor[1] + ", " + Rcolor[2]);
//            telemetry.addData("Lcolor: ", Lcolor[0] + ", " + Lcolor[1] + ", " + Lcolor[2]);
            telemetry.update();
        }
    }
}
