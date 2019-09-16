package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@TeleOp(name="SlideTest", group="Test")
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            double power = -gamepad1.left_stick_x;
            robot.RFmotor.setPower(power);
            telemetry.addData("A: RF:  ", robot.RFmotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
