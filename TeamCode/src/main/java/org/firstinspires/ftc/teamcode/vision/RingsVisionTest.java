package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RingsVisionTest", group="Test")
public class RingsVisionTest extends LinearOpMode {

    public void runOpMode() {
        RingsOpenCV vision = new RingsOpenCV(false, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            vision.analyzeRings(telemetry);
            sleep(50);
        }
    }

}
