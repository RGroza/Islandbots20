package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="OpticalDistTest", group="Test")
public class OpticalDistTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.update();
        }
    }
}
