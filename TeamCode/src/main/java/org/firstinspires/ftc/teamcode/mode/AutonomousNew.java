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

    private PIDController PIDHeadingCorrect = new PIDController(.01, .075, .15);
    private PIDController PIDRampSpeed = new PIDController(.1, 0, .5);
    private PIDController PIDTurn = new PIDController(.025, .001, .075);

    public void initPIDCorrection() {
        PIDHeadingCorrect.setOutputLimits(-.05, .05);
        PIDHeadingCorrect.setSetpoint(0);
    }

    public void initPIDRamping(double speed) {
        PIDRampSpeed.setOutputLimits(-speed, 0);
        PIDRampSpeed.setSetpoint(0);
    }

    public void initPIDTurning(double speed) {
        PIDTurn.setOutputLimits(-speed, speed);
        PIDTurn.setSetpoint(0);
    }

    public int detectAndGrabSkyStone(Telemetry telemetry) {
        VuforiaDetector vuforiaDetector = new VuforiaDetector(hardwareMap);
        waitForStart();

        int skyStoneVal = vuforiaDetector.runDetection(telemetry);

        telemetry.addData("skyStoneVal: ", skyStoneVal);
        telemetry.update();

        double currentAngle = robot.getPitch();

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        if (skyStoneVal == 0) {
            left(.5, 1.25, currentAngle, true);
            turnUntilPID(.5, currentAngle);
        } else if (skyStoneVal == 2) {
            right(.5, 1.25, currentAngle, true);
            turnUntilPID(.5, currentAngle);
        }

        moveUntilLaser(true, .4, 40, 5, currentAngle, true, false);

        robot.IntakeMotor.setPower(1);
        forward(.4, 3.5, currentAngle, true, false, telemetry);
        sleep(500);
        backward(.5, 3.5, currentAngle, true, false, telemetry);
        robot.IntakeMotor.setPower(0);

        robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);
        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_UP);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_UP);

        return skyStoneVal;
    }

    public void blueBlocksAuto(Telemetry telemetry) {
        int skyStoneVal = detectAndGrabSkyStone(telemetry);

        double currentAngle = robot.getPitch();

        turnUntilPID(.5, currentAngle - 90);

//        detectLineAndStop(false, false, .3, 10, currentAngle - 90, telemetry);
//        backward(.5, 5.5, true, false, telemetry);
        backward(.7, 10.5 + skyStoneVal, currentAngle - 90, true, false, telemetry);
        sleep(250);

        turnUntilPID(.5, currentAngle - 180);

        grabBlueFoundation(true, false, true, telemetry);

    }

    public void redBlocksAuto(Telemetry telemetry) {
        int skyStoneVal = detectAndGrabSkyStone(telemetry);

        double currentAngle = robot.getPitch();

        turnUntilPID(.5, currentAngle + 90);

//        detectLineAndStop(false, false, .3, 10, currentAngle + 90, telemetry);
//        backward(.5, 5.5, true, false, telemetry);
        backward(.7, 12.5 - skyStoneVal, currentAngle + 90, true, false, telemetry);
        sleep(250);

        turnUntilPID(.5, currentAngle + 180);

        grabRedFoundation(true, false, true, telemetry);

    }

    public void blueWallBlockAuto(Telemetry telemetry) {
        double currentAngle = robot.getPitch();

        blueFoundAuto(false, false, telemetry);
        right(.4, 2, currentAngle + 90, true);

        currentAngle = robot.getPitch();

        forward(.6, 14, currentAngle, true, false, telemetry);

        turnUntilPID(.5, currentAngle + 90);

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        moveUntilLaser(true, .3, 35, 5, currentAngle, true, false);

        currentAngle = robot.getPitch();

        turnUntilPID(.5, currentAngle - 30);

        robot.IntakeMotor.setPower(1);
        forward(.3, 1.5, currentAngle - 30, true, false, telemetry);
        sleep(500);
        turnUntilPID(.5, currentAngle);
        backward(.4, 3, currentAngle, true, false, telemetry);
        robot.IntakeMotor.setPower(0);
    }

    public void redWallBlockAuto(Telemetry telemetry) {
        double currentAngle = robot.getPitch();

        redFoundAuto(false, false, telemetry);
        left(.4, 2, currentAngle - 90, true);

        currentAngle = robot.getPitch();

        forward(.6, 14, currentAngle,true, false, telemetry);

        turnUntilPID(.5, currentAngle - 90);

        robot.LbeamServo.setPosition(CompetitionBot.L_BEAM_DOWN);
        robot.RbeamServo.setPosition(CompetitionBot.R_BEAM_DOWN);

        moveUntilLaser(true, .3, 35, 5, currentAngle - 90, true, false);

        currentAngle = robot.getPitch();

        turnUntilPID(.5, currentAngle + 30);

        robot.IntakeMotor.setPower(1);
        forward(.3, 1.5, currentAngle + 30, true, false, telemetry);
        sleep(500);
        turnUntilPID(.5, currentAngle);
        backward(.4, 3, currentAngle, true, false, telemetry);
        robot.IntakeMotor.setPower(0);
    }

    public void blueFoundAuto(boolean robotPark, boolean tapePark, Telemetry telemetry) {
        right(.4, 2, robot.getPitch(), true);
        grabBlueFoundation(false, robotPark, tapePark, telemetry);
    }

    public void redFoundAuto(boolean robotPark, boolean tapePark, Telemetry telemetry) {
        left(.4, 2, robot.getPitch(), true);
        grabRedFoundation(false, robotPark, tapePark, telemetry);
    }

    public void grabBlueFoundation(boolean depositBlock, boolean robotPark, boolean tapePark, Telemetry telemetry) {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        moveUntilLaser(false, .3, 10, 7.5, currentAngle, true, false); // using backDistance
        turnUntilPID(.5, currentAngle);

        backward(.3, .25, currentAngle, true, false, telemetry);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        if (depositBlock) {
            depositBlock();
        } else {
            forward(.5, 1, currentAngle, true, false, telemetry);
        }

        wideTurnUntilPID(.5, currentAngle + 90);
//        if (!depositBlock) {
//            robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
//            robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);
//            left(.5, 1, currentAngle + 90, true);
//            turnUntilPID(.5, currentAngle + 90);
//        }
        backward(.6, 1.75, currentAngle + 90, true, false, telemetry);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        if (tapePark) {
            robot.TapeMeasure.setPower(1);
            forward(.5, .5, currentAngle + 90, true, false, telemetry);
            turnByPID(.5, 15);
            sleep(1250);
            robot.TapeMeasure.setPower(0);
        } else {
            right(.6, 2.75, currentAngle + 90, true);
//            if (!depositBlock) left(.5, 2, currentAngle + 90, true);
            turnUntilPID(.5, currentAngle + 90);
            if (robotPark) {
                detectLineAndStop(true, true, .4, 10, currentAngle + 90, telemetry);
            } else {
                forward(.5, 10, currentAngle + 90, true, true, telemetry);
            }
        }
    }

    public void grabRedFoundation(boolean depositBlock, boolean robotPark, boolean tapePark, Telemetry telemetry) {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        turnUntilPID(.5, currentAngle);

        moveUntilLaser(false, .3, 10, 7.5, currentAngle, true, false); // using backDistance
        turnUntilPID(.5, currentAngle);

        backward(.3, .25, currentAngle, true, false, telemetry);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        if (depositBlock) {
            depositBlock();
        } else {
            forward(.5, 1, currentAngle, true, false, telemetry);
        }

        wideTurnUntilPID(.5, currentAngle - 90);
//        if (!depositBlock) {
//            robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
//            robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);
//            right(.5, 1, currentAngle - 90, true);
//            turnUntilPID(.5, currentAngle - 90);
//        }
        backward(.6, 1.75, currentAngle - 90, true, false, telemetry);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        if (tapePark) {
            robot.TapeMeasure.setPower(1);
            forward(.5, .5, currentAngle - 90, true, false, telemetry);
            turnByPID(.5, -15);
            sleep(1250);
            robot.TapeMeasure.setPower(0);
        } else {
            left(.6, 2.75, currentAngle - 90, true);
//            if (!depositBlock) left(.5, 2, currentAngle - 90, true);
            turnUntilPID(.5, currentAngle - 90);
            if (robotPark) {
                detectLineAndStop(true, true, .4, 10, currentAngle - 90, telemetry);
            } else {
                forward(.5, 10, currentAngle - 90, true, true, telemetry);
            }
        }
    }

    public void depositBlock() {
        robot.setMotors(.2, .2, .2, .2);

        robot.SlideMotor.setPower(.75);
        sleep(250);

        robot.SlideMotor.setTargetPosition(robot.SlideMotor.getCurrentPosition() + 10);
        robot.SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.SlideMotor.setPower(0);

        robot.armRotateServo.setPosition(CompetitionBot.ARM_OUT);
        sleep(500);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);
        sleep(500);
        robot.armRotateServo.setPosition(CompetitionBot.ARM_IN);

        robot.setMotors(0, 0, 0, 0);

        robot.SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void detectLineAndGyro(boolean isForward, int maxDist, ColorSensor colorSensor) {
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

        turnUntilPID(.3, headingCorrection);
    }

    public void detectLineAndStop(boolean isForward, boolean parkOnLine, double speed, double maxDist, double targetPitch, Telemetry telemetry) {
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

        initPIDCorrection();

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
            double posDiff = ((abs(currentPos - initPos)) / CompetitionBot.DRIVETAIN_PPR) + 2;
            detectLineAndStop(!isForward, false, .1, posDiff, targetPitch, telemetry);
        } else {
            if (abs(robot.getPitch() - initPitch) > .5) turnUntilPID(.5, targetPitch);
        }
    }

    public void detectLineAndContinue(boolean isForward, double speed, int maxDist, Telemetry telemetry) {
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

    public void turnBy(double speed, double deltaAngle) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double direction;
        double adjustedSpeed;
        double minSpeed = .15;

        while (opModeIsActive() && abs(diff) > .5) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);
            direction = diff > 0 ? 1 : -1;

            // adjust speed in increments
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? .2 : .3) : speed;

            robot.LFmotor.setPower(-direction * adjustedSpeed);
            robot.LBmotor.setPower(-direction * adjustedSpeed);
            robot.RFmotor.setPower(direction * adjustedSpeed);
            robot.RBmotor.setPower(direction * adjustedSpeed);

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("adjustedSpeed: ", adjustedSpeed);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void turnByPID(double maxSpeed, double deltaAngle) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double output;

        initPIDTurning(maxSpeed);

        while (opModeIsActive() && abs(diff) > .75) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            output = PIDTurn.getOutput(currentAngle, targetAngle);

            if (abs(output) < .075) break;

            robot.LFmotor.setPower(-output);
            robot.LBmotor.setPower(-output);
            robot.RFmotor.setPower(output);
            robot.RBmotor.setPower(output);

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("output: ", output);
            telemetry.update();

        }
        stopMotors(250);
    }

    public void wideTurnBy(double speed, double deltaAngle) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double adjustedSpeed;
        double minSpeed = .15;

        while (opModeIsActive() && abs(diff) > .75) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            // adjust speed in increments
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? .2 : .3) : speed;

            if (diff > 0) {
                robot.LFmotor.setPower(-minSpeed);
                robot.LBmotor.setPower(-minSpeed);
                robot.RFmotor.setPower(adjustedSpeed);
                robot.RBmotor.setPower(adjustedSpeed);
            } else {
                robot.LFmotor.setPower(adjustedSpeed);
                robot.LBmotor.setPower(adjustedSpeed);
                robot.RFmotor.setPower(-minSpeed);
                robot.RBmotor.setPower(-minSpeed);
            }

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("adjustedSpeed: ", adjustedSpeed);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void wideTurnByPID(double maxSpeed, double deltaAngle) {
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double output;
        double minSpeed = .15;

        initPIDTurning(maxSpeed);

        while (opModeIsActive() && abs(diff) > .5) {
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);

            output = PIDTurn.getOutput(currentAngle, targetAngle);

            if (abs(output) < .075) break;

            if (abs(diff) > 5) {
                if (diff > 0) {
                    robot.LFmotor.setPower(-minSpeed);
                    robot.LBmotor.setPower(-minSpeed);
                    robot.RFmotor.setPower(output);
                    robot.RBmotor.setPower(output);
                } else {
                    robot.LFmotor.setPower(-output);
                    robot.LBmotor.setPower(-output);
                    robot.RFmotor.setPower(-minSpeed);
                    robot.RBmotor.setPower(-minSpeed);
                }
            } else {
                robot.LFmotor.setPower(-output);
                robot.LBmotor.setPower(-output);
                robot.RFmotor.setPower(output);
                robot.RBmotor.setPower(output);
            }

            telemetry.addData("gyro: ", robot.getPitch());
            telemetry.addData("diff: ", diff);
            telemetry.addData("output: ", output);
            telemetry.update();

        }
        stopMotors(250);
    }

    public void turnUntil(double speed, double absAngle) {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        turnBy(speed, diff);
    }

    public void turnUntilPID(double speed, double absAngle) {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        turnByPID(speed, diff);
    }

    public void wideTurnUntil(double speed, double absAngle) {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        wideTurnBy(speed, diff);
    }

    public void wideTurnUntilPID(double speed, double absAngle) {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        wideTurnByPID(speed, diff);
    }

    private double rampSpeed(double currentVal, double initVal, double targetVal, double speed, double minSpeed, boolean linearRamp) {
        double range = abs(targetVal - initVal);
        double currentDist = abs(currentVal - initVal);
        double remainingDist = abs(targetVal - currentVal);
        double rampRange = .25*range;

        telemetry.addData("dist: ", Math.round(currentDist / range * 100.0) / 100.0);

        if (currentDist <= range) {
            if (remainingDist <= rampRange) {
                return (remainingDist / rampRange) * (speed - minSpeed) * (linearRamp ? 1 : (speed - minSpeed)) + minSpeed;
            }
        } else {
            stopMotors(250);
            return 0;
        }
        return speed;
    }

    private double rampPID(double currentVal, double initVal, double targetVal, double speed, Telemetry telemetry) {
        double range = abs(targetVal - initVal);
        double currentDist = abs(currentVal - initVal);
//        double rampRange = .25*range;

        telemetry.addData("dist: ", Math.round(currentDist / range * 100.0) / 100.0);

        if (currentVal < targetVal) {
            return speed + PIDRampSpeed.getOutput(currentVal, targetVal);
//            if (abs(targetVal - currentVal) <= rampRange) {
//                return speed + PIDRampSpeed.getOutput(currentVal, targetVal);
//            } else if (currentVal >= initVal && currentVal < targetVal - rampRange) {
//                return speed;
//            }
        }
        stopMotors(250);
        return 0;
    }

    private void stopMotors(int durationTime) {
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

        robot.setMotors(0, 0, 0, 0);

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        if (frontSensor) {
            if (robot.frontDistance.getDistance(DistanceUnit.CM) - distance > 0) {
                forwardUntilLaser(frontSensor, speed, distance, maxRevs, targetPitch, PIDCorrect, PIDRamp);
            } else {
                backwardUntilLaser(frontSensor, speed, distance, maxRevs, targetPitch, PIDCorrect, PIDRamp);
            }
        } else {
            if (robot.backDistance.getDistance(DistanceUnit.CM) - distance > 0) {
                backwardUntilLaser(frontSensor, speed, distance, maxRevs, targetPitch, PIDCorrect, PIDRamp);
            } else {
                forwardUntilLaser(frontSensor, speed, distance, maxRevs, targetPitch, PIDCorrect, PIDRamp);
            }
        }
    }

    public void moveUntilSonar(double speed, double voltage, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        if (voltage - robot.sonarDistance.getVoltage() > 0) {
            backwardUntilSonar(speed, voltage, maxRevs, targetPitch, PIDCorrect, PIDRamp);
        } else {
            forwardUntilSonar(speed, voltage, maxRevs, targetPitch, PIDCorrect, PIDRamp);
        }
    }

    public void forward(double speed, double revCount, double targetPitch, boolean PIDCorrect, boolean PIDRamp, Telemetry telemetry) {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos + stepCount;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) avgMotorPos();

            if (PIDRamp) {
                rampedSpeed = rampPID(avgPos, initPos, targetPos, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .05, true);
            }
            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void forwardUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        double currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                        : robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                     : robot.backDistance.getDistance(DistanceUnit.CM);

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .05, true);
            }

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void forwardUntilSonar(double speed, double voltage, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .05 ? robot.sonarDistance.getVoltage() : currentVal;

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .05, true);
            }

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void backward(double speed, double revCount, double targetPitch, boolean PIDCorrect, boolean PIDRamp, Telemetry telemetry) {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) avgMotorPos();
        int initPos = avgPos;
        int targetPos = avgPos - stepCount;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && avgPos > targetPos) {
            avgPos = (int) avgMotorPos();

            if (PIDRamp) {
                rampedSpeed = rampPID(avgPos, initPos, targetPos, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .05, true);
            }
            telemetry.addData("rampedSpeed: ", rampedSpeed);
            telemetry.update();

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void backwardUntilLaser(boolean frontSensor, double speed, double distance, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        double currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                        : robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = frontSensor ? robot.frontDistance.getDistance(DistanceUnit.CM)
                                     : robot.backDistance.getDistance(DistanceUnit.CM);

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .05, true);
            }

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void backwardUntilSonar(double speed, double voltage, double maxRevs, double targetPitch, boolean PIDCorrect, boolean PIDRamp) {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }
        if (PIDRamp) {
            initPIDRamping(speed);
        }

        double rampedSpeed;
        while (opModeIsActive() && (currentVal - targetVal) > .5 && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .075 ? robot.sonarDistance.getVoltage() : currentVal;

            if (PIDRamp) {
                rampedSpeed = rampPID(currentVal, initVal, targetVal, speed, telemetry);
            } else {
                rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .05, true);
            }

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void left(double speed, double revCount, double targetPitch, boolean PIDCorrect) {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        int targetPos = avgPos + stepCount;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }

        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    public void right(double speed, double revCount, double targetPitch, boolean PIDCorrect) {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int targetPos = avgPos + stepCount;

        double output;

        if (PIDCorrect) {
            initPIDCorrection();
        }

        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);

            if (PIDCorrect) {
                output = PIDHeadingCorrect.getOutput(robot.getPitch(), targetPitch);
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

    private double avgMotorPos() {
        return ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
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
