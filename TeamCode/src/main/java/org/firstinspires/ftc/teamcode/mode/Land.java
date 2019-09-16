package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Land", group="Competition")
public class Land extends AutonomousNew {
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new CompetitionBot(hardwareMap, telemetry);
//        robot.mineralLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        if(opModeIsActive()) {
//            land();
        }
    }
}
