package org.firstinspires.ftc.teamcode.mode;

import org.firstinspires.ftc.teamcode.vision.RingsOpenCV;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RingsAuto", group="Autonomous")
public class RingsAuto extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        RingsOpenCV vision = new RingsOpenCV(hardwareMap, telemetry);

        waitForStart();
        telemetry.addData("Analysis", vision.getAnalysis());
        telemetry.addData("Position", vision.getPosition());
    }
}
