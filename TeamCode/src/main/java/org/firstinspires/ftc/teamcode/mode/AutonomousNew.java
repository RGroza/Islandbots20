package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

// VUFORIA IMPORTS

import org.firstinspires.ftc.teamcode.robot.VuforiaDetector;


public abstract class AutonomousNew extends LinearOpMode {
    protected CompetitionBot robot;

    private double gyroCorrectConst = .02;

    private PIDController PIDHeadingCorrect = new PIDController(.01, .075, .15);
    private PIDController PIDRampSpeed = new PIDController(.5, 0, .2);

    public void initPIDCorrection(Telemetry telemetry, PIDController PID) {
        PID.setOutputLimits(-.05, .05);
        PID.setSetpoint(0);
    }

    public void initPIDRamping(Telemetry telemetry, double speed, PIDController PID) {
        PID.setOutputLimits(-speed, 0);
        PID.setSetpoint(0);
    }

    public void detectAndGrabSkyStone(Telemetry telemetry) throws InterruptedException {
        VuforiaDetector vuforiaDetector = new VuforiaDetector(hardwareMap);
        waitForStart();

        int skyStoneVal = vuforiaDetector.runDetection(telemetry);

        telemetry.addData("skyStoneVal: ", skyStoneVal);
        telemetry.update();

        double currentAngle = robot.getPitch();

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        if (skyStoneVal == 0) {
            left(.4, 2.5, true);
            turnUntil(.5, currentAngle);
        } else if (skyStoneVal == 2) {
            right(.4, 2.5, true);
            turnUntil(.5, currentAngle);
        }

        moveUntilLaser(true, .3, 30, 5, true, false);
        turnUntil(.5, currentAngle);

        robot.IntakeMotor.setPower(1);
        forward(.3, 3, true, false, telemetry);
        sleep(500);
        backward(.4, 3, true, false, telemetry);
        robot.IntakeMotor.setPower(0);

        robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);
        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_UP);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_UP);
    }

    public void blueBlocksAuto(Telemetry telemetry) throws InterruptedException {
        detectAndGrabSkyStone(telemetry);

        double currentAngle = robot.getPitch();

        turnUntil(.4, currentAngle - 90);

//        detectLineAndStop(false, false, .3, 10, currentAngle - 90, telemetry);
//        backward(.5, 5.5, true, false, telemetry);
        backward(.5, 11.5, true, false, telemetry);
        sleep(250);

        turnUntil(.5, currentAngle - 180);

        grabBlueFoundation(true, true, telemetry);

    }

    public void redBlocksAuto(Telemetry telemetry) throws InterruptedException {
        detectAndGrabSkyStone(telemetry);

        double currentAngle = robot.getPitch();

        turnUntil(.4, currentAngle + 90);

//        detectLineAndStop(false, false, .3, 10, currentAngle + 90, telemetry);
//        backward(.5, 5.5, true, false, telemetry);
        backward(.5, 11.5, true, false, telemetry);
        sleep(250);

        turnUntil(.5, currentAngle + 180);

        grabRedFoundation(true, true, telemetry);

    }

    public void blueWallBlockAuto(Telemetry telemetry) throws InterruptedException {
        blueFoundAuto(false, telemetry);
        right(.4, 2, true);

        double currentAngle = robot.getPitch();

        forward(.6, 14, true, false, telemetry);

        turnUntil(.5, currentAngle + 90);

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        moveUntilLaser(true, .3, 35, 5, true, false);

        currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle - 30);

        robot.IntakeMotor.setPower(1);
        forward(.3, 1.5, true, false, telemetry);
        sleep(500);
        turnUntil(.5, currentAngle);
        backward(.4, 3, true, false, telemetry);
        robot.IntakeMotor.setPower(0);
    }

    public void redWallBlockAuto(Telemetry telemetry) throws InterruptedException {
        redFoundAuto(false, telemetry);
        left(.4, 2, true);

        double currentAngle = robot.getPitch();

        forward(.6, 14, true, false, telemetry);

        turnUntil(.5, currentAngle - 90);

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        moveUntilLaser(true, .3, 35, 5, true, false);

        currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle + 30);

        robot.IntakeMotor.setPower(1);
        forward(.3, 1.5, true, false, telemetry);
        sleep(500);
        turnUntil(.5, currentAngle);
        backward(.4, 3, true, false, telemetry);
        robot.IntakeMotor.setPower(0);
    }

    public void blueFoundAuto(boolean park, Telemetry telemetry) throws InterruptedException {
        right(.4, 1.5, true);
        grabBlueFoundation(false, park, telemetry);
    }

    public void redFoundAuto(boolean park, Telemetry telemetry) throws InterruptedException {
        left(.4, 1.5, true);
        grabRedFoundation(false, park, telemetry);
    }

    public void grabBlueFoundation(boolean depositBlock, boolean park, Telemetry telemetry) throws InterruptedException {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        moveUntilLaser(false, .3, 7.5, 7.5, true, false); // using backDistance
        turnUntil(.5, currentAngle);

        backward(.3, .25, true, false, telemetry);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        turnUntil(.5, currentAngle);

        if (depositBlock) depositBlock(telemetry);

//        forward(.5, .5, true, false, telemetry);
        sleep(250);
        turnUntil(.5, currentAngle + 90);
        backward(.5, 1, false, false, telemetry);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        if (park) {
            robot.TapeMeasure.setPower(1);
            forward(.5, .5, true, false, telemetry);
            turnBy(.5, 20);
            sleep(1250);
            robot.TapeMeasure.setPower(0);
        }
    }

    public void grabRedFoundation(boolean depositBlock, boolean park, Telemetry telemetry) throws InterruptedException {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        moveUntilLaser(false, .3, 7.5, 7.5, true, false); // using backDistance
        turnUntil(.5, currentAngle);

        backward(.3, .25, true, false, telemetry);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        turnUntil(.5, currentAngle);

        if (depositBlock) depositBlock(telemetry);

//        forward(.5, .5, true, false, telemetry);
        sleep(250);
        turnUntil(.5, currentAngle - 90);
        backward(.5, 1, false, false, telemetry);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        if (park) {
            robot.TapeMeasure.setPower(1);
            forward(.5, .5, true, false, telemetry);
            turnBy(.5, -20);
            sleep(1250);
            robot.TapeMeasure.setPower(0);
        }
    }

    public void depositBlock(Telemetry telemetry) throws InterruptedException {
        robot.setMotors(.3, .3, .3, .3);

        robot.SlideMotor.setPower(.75);
        sleep(250);

        robot.SlideMotor.setTargetPosition(robot.SlideMotor.getCurrentPosition() + 10);
        robot.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.SlideMotor.setPower(0);

        robot.armRotateServo.setPosition(CompetitionBot.ARM_OUT);
        sleep(300);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);
        sleep(500);
        robot.armRotateServo.setPosition(CompetitionBot.ARM_IN);

        robot.setMotors(0, 0, 0, 0);

        robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void detectLineAndGyro(boolean isForward, int maxDist, ColorSensor colorSensor, Telemetry telemetry) throws InterruptedException {
        final int BLUE_THRESHOLD = 200;
        final int RED_THRESHOLD = 200;

        int initialPos = (int) avgMotorPos();
        int currentPos = initialPos;

        int headingCorrection = robot.getPitch() > Math.abs(robot.getPitch() - 180) ? 0 : 180;

        while (opModeIsActive() && (colorSensor.blue() < BLUE_THRESHOLD || colorSensor.red() < RED_THRESHOLD)
                && Math.abs(currentPos - initialPos) > maxDist && opModeIsActive()) {
            double power = isForward ? .3 : -.3;
            robot.setMotors(power, power, power, power);
        }

        turnUntil(.3, headingCorrection);
    }

    public void detectLineAndStop(boolean isForward, boolean parkOnLine, double speed, double maxDist, double targetPitch, Telemetry telemetry) throws InterruptedException {
        NormalizedRGBA colorsL = robot.LcolorSensor.getNormalizedColors();
        NormalizedRGBA colorsR = robot.RcolorSensor.getNormalizedColors();
        telemetry.addData("Left R: ",  colorsL.red);
        telemetry.addData("Right R: ",  colorsR.red);
        telemetry.addData("Left G: ",  colorsL.green);
        telemetry.addData("Right G: ",  colorsR.green);
        telemetry.addData("Left B: ",  colorsL.blue);
        telemetry.addData("Right B: ",  colorsR.blue);
        telemetry.addData("Left A: ",  colorsL.alpha);
        telemetry.addData("Right A: ",  colorsR.alpha);
        telemetry.update();
        final double R_COLOR_THRESHOLD = .02;
        final double  L_COLOR_THRESHOLD = .02;
        double RSpeed = speed;
        double LSpeed = speed;

        int maxSteps = (int) (maxDist*CompetitionBot.DRIVETAIN_PPR);

        int dir = 1;

        if (!isForward) {
            dir = -1;
        }

        int initialPosition = (int) avgMotorPos();
        int currentPosition = initialPosition;
        int maxPos = initialPosition + maxSteps;

        double initPitch = robot.getPitch();

        double actualPitch = initPitch;
        double output;

        initPIDCorrection(telemetry, PIDHeadingCorrect);

        boolean LStop = false, RStop = false;
        int stopCount = 0;
        while (opModeIsActive() && abs(currentPosition - initialPosition) < maxPos && (!LStop && !RStop)) {
            currentPosition = (int) avgMotorPos();
            colorsL = robot.LcolorSensor.getNormalizedColors();
            colorsR = robot.RcolorSensor.getNormalizedColors();

            if (!LStop && colorsL.blue > L_COLOR_THRESHOLD || colorsL.red > L_COLOR_THRESHOLD) {
                LSpeed = 0;
                LStop = true;
                stopCount++;
            }
            if (!RStop && colorsR.blue > R_COLOR_THRESHOLD || colorsR.red > R_COLOR_THRESHOLD) {
                RSpeed = 0;
                RStop = true;
                stopCount++;
            }

            if (stopCount == 1 && abs(robot.getPitch() - initPitch) > 5) {
                robot.setMotors(0, 0, 0, 0);
                break;
            }

            actualPitch = robot.getPitch();
            output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);

            robot.setMotors(dir*LSpeed - output, dir*LSpeed - output, dir*RSpeed + output, dir*RSpeed + output);
        }

        int initPos = (int) avgMotorPos();
        stopMotors(250);
        int currentPos = (int) avgMotorPos();

        // If robot overshoots, return to line in second pass
        if (parkOnLine && abs(currentPos - initPos) > 100) {
            double posDiff = ((abs(currentPos - initPos)) / CompetitionBot.DRIVETAIN_PPR) + 1;
            detectLineAndStop(!isForward, false, .1, posDiff, targetPitch, telemetry);
        } else {
            turnUntil(.5, targetPitch);
        }
    }

    public void detectLineAndContinue(boolean isForward, double speed, int maxDist, Telemetry telemetry) throws InterruptedException {
        NormalizedRGBA colorsL = robot.LcolorSensor.getNormalizedColors();
        NormalizedRGBA colorsR = robot.RcolorSensor.getNormalizedColors();
        telemetry.addData("Left R: ",  colorsL.red);
        telemetry.addData("Right R: ",  colorsR.red);
        telemetry.addData("Left G: ",  colorsL.green);
        telemetry.addData("Right G: ",  colorsR.green);
        telemetry.addData("Left B: ",  colorsL.blue);
        telemetry.addData("Right B: ",  colorsR.blue);
        telemetry.addData("Left A: ",  colorsL.alpha);
        telemetry.addData("Right A: ",  colorsR.alpha);
        telemetry.update();
        final double R_COLOR_THRESHOLD = .02;
        final double  L_COLOR_THRESHOLD = .02;
        double RSpeed = speed;
        double LSpeed = speed;

        int maxSteps = (int) (maxDist*CompetitionBot.DRIVETAIN_PPR);

        int dir = 1;

        if (!isForward) {
            dir = -1;
        }

        int initialPosition = (int) avgMotorPos();
        int currentPosition = initialPosition;
        int maxPos = initialPosition + maxSteps;

        boolean LStop = false, RStop = false;
        while(opModeIsActive() && abs(currentPosition - initialPosition) < maxPos && (!LStop && !RStop)) {
            currentPosition = (int) avgMotorPos();
            colorsL = robot.LcolorSensor.getNormalizedColors();
            colorsR = robot.RcolorSensor.getNormalizedColors();

            if (colorsL.blue > L_COLOR_THRESHOLD || colorsL.red > L_COLOR_THRESHOLD) {
                LSpeed = 0;
                LStop = true;
            }
            if (colorsR.blue > R_COLOR_THRESHOLD || colorsR.red > R_COLOR_THRESHOLD) {
                RSpeed = 0;
                RStop = true;
            }

            robot.setMotors(dir*LSpeed, dir*LSpeed, dir*RSpeed, dir*RSpeed);
        }
    }

    public void turnBy(double speed, double deltaAngle) throws InterruptedException {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double diffDecimal;
        double direction;
        double adjustedSpeed;
        double minSpeed = .15;

        while (opModeIsActive() && abs(diff) > .5) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);
            diffDecimal = diff / deltaAngle;
            direction = diff > 0 ? 1 : -1;

            // adjust speed proportionally
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? .2 : .3) : speed;

            robot.LFmotor.setPower(-direction * adjustedSpeed);
            robot.LBmotor.setPower(-direction * adjustedSpeed);
            robot.RFmotor.setPower(direction * adjustedSpeed);
            robot.RBmotor.setPower(direction * adjustedSpeed);

            telemetry.addData("Gyro: ", robot.getPitch());
            telemetry.addData("Diff: ", diff);
            telemetry.addData("diffPercent: ", diffDecimal);
            telemetry.addData("adjustedSpeed: ", adjustedSpeed);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void wideTurnBy(double speed, double deltaAngle) throws InterruptedException {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double diffDecimal;
        double direction;
        double adjustedSpeed;
        double minSpeed = .15;

        while (opModeIsActive() && abs(diff) > .5) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);
            diffDecimal = diff / deltaAngle;
            direction = diff > 0 ? 1 : -1;

            // adjust speed proportionally
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? .2 : .3) : speed;

            if (diff > 0) {
                robot.LFmotor.setPower(-direction * minSpeed);
                robot.LBmotor.setPower(-direction * minSpeed);
                robot.RFmotor.setPower(direction * adjustedSpeed);
                robot.RBmotor.setPower(direction * adjustedSpeed);
            } else {
                robot.LFmotor.setPower(-direction * adjustedSpeed);
                robot.LBmotor.setPower(-direction * adjustedSpeed);
                robot.RFmotor.setPower(direction * minSpeed);
                robot.RBmotor.setPower(direction * minSpeed);
            }

            telemetry.addData("Gyro: ", robot.getPitch());
            telemetry.addData("Diff: ", diff);
            telemetry.addData("diffPercent: ", diffDecimal);
            telemetry.addData("adjustedSpeed: ", adjustedSpeed);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void turnUntil(double speed, double absAngle) throws InterruptedException {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        turnBy(speed, diff);
    }

    public void wideTurnUntil(double speed, double absAngle) throws InterruptedException {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        wideTurnBy(speed, diff);
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

    private double ramp(double currentDistance, double distanceTarget, double speed) {
        double MIN_SPEED = 0.15;
        double RAMP_DIST = .2; // Distance over which to do ramping
        // make sure speed is positive
        speed = abs(speed);
        double deltaSpeed = speed - MIN_SPEED;
        if (deltaSpeed <= 0) { // requested speed is below minimal
            return MIN_SPEED;
        }

        double currentDeltaDistance = abs(distanceTarget - currentDistance);
        // compute the desired speed
        if(currentDeltaDistance < RAMP_DIST) {
            return MIN_SPEED + (deltaSpeed * (currentDeltaDistance / RAMP_DIST));
        }
        return speed; // default
    }

    private double ramp(int position, int deltaDistance, int finalPosition, double speed) {
        // dynamically adjust speed based on encoder values
        double MIN_SPEED = 0.2;
        double RAMP_DIST = 300; // Distance over which to do ramping
        // make sure speed is positive
        speed = abs(speed);

        double deltaSpeed = speed - MIN_SPEED;
        if (deltaSpeed <= 0) { // requested speed is below minimal
            return MIN_SPEED;
        }
        // adjust ramping distance for short distances
        if (abs(deltaDistance) < 3 * RAMP_DIST) {
            RAMP_DIST = deltaDistance / 3;
        }

        int currentDeltaDistance = abs(position - (finalPosition - deltaDistance));
        // now compute the desired speed
        if (currentDeltaDistance < RAMP_DIST) {
            return MIN_SPEED + deltaSpeed * (currentDeltaDistance / RAMP_DIST);
        } else if (currentDeltaDistance > abs(deltaDistance) - RAMP_DIST * 2) {
            return MIN_SPEED + (deltaSpeed * (abs(finalPosition - position)) / (RAMP_DIST * 2));
        } else if (currentDeltaDistance > abs(deltaDistance)) { // overshoot
            return 0;
        }

        return speed; // default
    }

    private double rampSpeed(double currentVal, double initVal, double targetVal, double speed, double minSpeed, boolean linearRamp) {
        double range = targetVal - initVal;
        double brakeRange = .2*range;
        double rampRange = .15*range;

        if (abs(currentVal - initVal) < abs(range)) {
//            if (abs(currentVal - initVal) <= rampRange) {
//                return ((currentVal - initVal) / rampRange) * (speed - minSpeed) * (linearRamp ? 1 : (speed - minSpeed)) + minSpeed;
//            }
            if (abs(targetVal - currentVal) <= brakeRange) {
                if (abs(targetVal - currentVal) <= rampRange) {
                    return ((targetVal - currentVal) / rampRange) * (speed - minSpeed) * (linearRamp ? 1 : (speed - minSpeed)) + minSpeed;
                } else {
                    return 0;
                }
            }
        } else {
            return 0;
        }
        return speed;
    }

    private double rampPID(double currentVal, double initVal, double targetVal, double speed) {
        double range = targetVal - initVal;
        double rampRange = .25*range;

        if (currentVal < targetVal) {
            if (abs(targetVal - currentVal) <= rampRange) {
                return speed + PIDRampSpeed.getOutput(currentVal, targetVal);
            } else if (currentVal >= initVal && currentVal < targetVal - rampRange) {
                return speed;
            }
        }
        return 0;
    }

    private double avgMotorPos() throws InterruptedException {
        return ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
    }

    private void stopMotors(int durationTime) throws InterruptedException {
        double currentAngle = robot.getPitch();

        robot.LFmotor.setTargetPosition(robot.LFmotor.getCurrentPosition());
        robot.LBmotor.setTargetPosition(robot.LBmotor.getCurrentPosition());
        robot.RFmotor.setTargetPosition(robot.RFmotor.getCurrentPosition());
        robot.RBmotor.setTargetPosition(robot.RBmotor.getCurrentPosition());

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotors(.75, .75, .75, .75);

        sleep(durationTime);

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turnUntil(.5, currentAngle);
    }

    public void moveUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        if (frontSensor) {
            if (robot.frontDistance.getDistance(DistanceUnit.CM) - distance > 0) {
                forwardUntilLaser(frontSensor, speed, distance, maxRevs, PIDCorrect, PIDRamp);
            } else {
                backwardUntilLaser(frontSensor, speed, distance, maxRevs, PIDCorrect, PIDRamp);
            }
        } else {
            if (robot.backDistance.getDistance(DistanceUnit.CM) - distance > 0) {
                backwardUntilLaser(frontSensor, speed, distance, maxRevs, PIDCorrect, PIDRamp);
            } else {
                forwardUntilLaser(frontSensor, speed, distance, maxRevs, PIDCorrect, PIDRamp);
            }
        }
    }

    public void moveUntilSonar(double speed, double voltage, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        if (voltage - robot.sonarDistance.getVoltage() > 0) {
            backwardUntilSonar(speed, voltage, maxRevs, PIDCorrect, PIDRamp);
        } else {
            forwardUntilSonar(speed, voltage, maxRevs, PIDCorrect, PIDRamp);
        }
    }

    public void forward(double speed, double revCount, boolean PIDCorrect, boolean PIDRamp, Telemetry telemetry) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos + stepCount;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) avgMotorPos();

            if (PIDRamp) {
                rampedSpeed = rampPID(avgPos, initPos, targetPos, speed);
            } else {
                rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, false);
            }
            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                                clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                robot.setMotors(clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void forwardUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        double currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                        : robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                     : robot.backDistance.getDistance(DistanceUnit.CM);

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);
            }

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                                clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                robot.setMotors(clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void forwardUntilSonar(double speed, double voltage, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .05 ? robot.sonarDistance.getVoltage() : currentVal;

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);
            }

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                        clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                robot.setMotors(clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed),
                                clamp(rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backward(double speed, double revCount, boolean PIDCorrect, boolean PIDRamp, Telemetry telemetry) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos - stepCount;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && avgPos > targetPos) {
            avgPos = (int) avgMotorPos();

            if (PIDRamp) {
                rampedSpeed = rampPID(avgPos, initPos, targetPos, speed);
            } else {
                rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, false);
            }
            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                                clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                robot.setMotors(clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backwardUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        double currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                        : robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                     : robot.backDistance.getDistance(DistanceUnit.CM);

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);
            }

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                        clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                robot.setMotors(clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backwardUntilSonar(double speed, double voltage, double maxRevs, boolean PIDCorrect, boolean PIDRamp) throws InterruptedException {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }
        if (PIDRamp) {
            initPIDRamping(telemetry, speed, PIDRampSpeed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .075 ? robot.sonarDistance.getVoltage() : currentVal;

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);
            }

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                        clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                robot.setMotors(clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed),
                                clamp(-rampedSpeed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void left(double speed, double revCount, boolean PIDCorrect) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        int initPos = avgPos;
        int targetPos = avgPos + stepCount;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }

        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-speed - output), clamp(speed - output),
                                clamp(speed + output), clamp(-speed + output));
            } else {
                robot.setMotors(clamp(-speed),
                                clamp(speed),
                                clamp(speed),
                                clamp(-speed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

/*
    public void leftUntil(double speed, double distance, double maxRevs, boolean PID) throws InterruptedException {
        double currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        while (opModeIsActive() && currentDistance >= distance && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-speed - output), clamp(speed - output),
                                clamp(speed + output), clamp(-speed + output));
            } else {
                robot.setMotors(clamp(-speed),
                                clamp(speed),
                                clamp(speed),
                                clamp(-speed));
            }
        }
        robot.setMotors(0,0,0,0);
    }
*/

    public void right(double speed, double revCount, boolean PIDCorrect) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int initPos = avgPos;
        int targetPos = avgPos + stepCount;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PIDCorrect) {
            initPIDCorrection(telemetry, PIDHeadingCorrect);
        }

        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);

            if (PIDCorrect) {
                actualPitch = robot.getPitch();
                output = PIDHeadingCorrect.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(speed - output), clamp(-speed - output),
                        clamp(-speed + output), clamp(speed + output));
            } else {
                robot.setMotors(clamp(speed),
                                clamp(-speed),
                                clamp(-speed),
                                clamp(speed));
            }
        }
        robot.setMotors(0,0,0,0);
    }

/*
    public void rightUntil(double speed, double distance, double maxRevs, boolean PID) throws InterruptedException {
        double currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        while (opModeIsActive() && currentDistance >= distance && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(speed - output), clamp(-speed - output),
                                clamp(-speed + output), clamp(speed + output));
            } else {
                robot.setMotors(clamp(speed),
                                clamp(-speed),
                                clamp(-speed),
                                clamp(speed));
            }
        }
        robot.setMotors(0,0,0,0);
    }
*/

    private double gyroCorrect(double targetPitch, double currentPitch) {
        double diff = angleDiff(currentPitch, targetPitch);
        if(abs(diff) < 1) {
            diff = 0;
        }
        return (diff * gyroCorrectConst);
    }

    private double angleDiff(double angle1, double angle2) {
        double d1 = angle2 - angle1;
        if (d1 > 180) {
            return d1 - 360;
        } else if (d1 < -180) {
            return d1 + 360;
        } else {
            return d1;
        }
    }

}
