package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;

import org.firstinspires.ftc.teamcode.robot.MiniPID;

public class PIDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        MiniPID miniPID;

        miniPID = new MiniPID(0.25, 0.01, 0.4);
        miniPID.setOutputLimits(-1, 1);
        //miniPID.setMaxIOutput(2);
        //miniPID.setOutputRampRate(3);
        //miniPID.setOutputFilter(.3);
        //miniPID.setSetpointRange(40);

        double actual = robot.getPitch();
        double target = actual;

        double output;

        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);

        waitForStart();
        while (opModeIsActive()) {

            actual = robot.getPitch();
            output = miniPID.getOutput(actual, target);

            telemetry.addData("actual: ", actual);
            telemetry.addData("output: ", output);

        }
    }
}
