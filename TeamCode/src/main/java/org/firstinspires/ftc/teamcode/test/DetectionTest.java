package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.VuforiaDetector;

@Autonomous(name="DetectionTest", group="Test")
public class DetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        VuforiaDetector vuforiaDetector = new VuforiaDetector(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            vuforiaDetector.runDetection(telemetry);
            sleep(1000);
        }
    }
}
