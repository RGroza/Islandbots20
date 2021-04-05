package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@Autonomous(name="HighGoalVisionTest", group="Test")
public class HighGoalVisionTest extends LinearOpMode {

    public void runOpMode() {
        RingsOpenCV vision = new RingsOpenCV(false, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            vision.analyzeRings(telemetry);
            sleep(50);
        }
    }

}
