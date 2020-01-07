package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.GamepadButton;
import org.firstinspires.ftc.teamcode.robot.MiniPID;

@Autonomous(name="PIDTest", group="Test")
public class PIDTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CompetitionBot robot = new CompetitionBot(hardwareMap, telemetry);

        GamepadButton forwardToggle = new GamepadButton(300, false);
        GamepadButton backwardToggle = new GamepadButton(300, false);
        GamepadButton leftToggle = new GamepadButton(300, false);
        GamepadButton rightToggle = new GamepadButton(300, false);

        GamepadButton speedIncButton = new GamepadButton(500, false);
        GamepadButton speedDecButton = new GamepadButton(500, false);

        MiniPID miniPID;

        miniPID = new MiniPID(.01, .075, .15);
        miniPID.setOutputLimits(-.05, .05);
        //miniPID.setMaxIOutput(2);
        //miniPID.setOutputRampRate(3);
        //miniPID.setOutputFilter(.3);
        //miniPID.setSetpointRange(40);

        double actual = robot.getPitch();
        double target = actual;

        double output;

        miniPID.setSetpoint(0);
        miniPID.setSetpoint(target);

        double speed = .25;

        waitForStart();
        while (opModeIsActive()) {

            boolean forwardBool = gamepad1.y;
            boolean backwardBool = gamepad1.a;
            boolean leftBool = gamepad1.x;
            boolean rightBool = gamepad1.b;

            boolean speedIncBool = gamepad1.dpad_up;
            boolean speedDecBool = gamepad1.dpad_down;

            forwardToggle.checkStatus(forwardBool);
            backwardToggle.checkStatus(backwardBool);
            leftToggle.checkStatus(leftBool);
            rightToggle.checkStatus(rightBool);

            speedIncButton.checkStatus(speedIncBool);
            speedDecButton.checkStatus(speedDecBool);

            actual = robot.getPitch();
            output = miniPID.getOutput(actual, target);

            if (forwardToggle.pressed) {
                robot.setMotors(clamp(speed - output), clamp(speed - output),
                                clamp(speed + output), clamp(speed + output));
            } else if (backwardToggle.pressed){
                robot.setMotors(clamp(-speed - output), clamp(-speed - output),
                                clamp(-speed + output), clamp(-speed + output));
            } else if (leftToggle.pressed){
                robot.setMotors(clamp(-speed - output), clamp(speed - output),
                                clamp(speed + output), clamp(-speed + output));
            } else if (rightToggle.pressed){
                robot.setMotors(clamp(speed - output), clamp(-speed - output),
                                clamp(-speed + output), clamp(speed + output));
            } else {
                robot.setMotors(0, 0, 0, 0);
            }

            if (speedIncButton.justPressed) { speed += .05; }
            if (speedDecButton.justPressed) { speed -= .05; }

            telemetry.addData("actual: ", actual);
            telemetry.addData("output: ", output);
            telemetry.addData("speed: ", speed);
            telemetry.addData("LF: ", robot.LFmotor.getPower());
            telemetry.addData("LR: ", robot.LBmotor.getPower());
            telemetry.addData("RF: ", robot.RFmotor.getPower());
            telemetry.addData("RB: ", robot.RBmotor.getPower());
            telemetry.update();

        }
    }

    private double clamp(double power) {
        // ensures power does not exceed abs(1)
        if (power > 1) {
            return 1;
        }
        if (power < -1) {
            return -1;
        }
        return power;
    }
}
