package org.firstinspires.ftc.teamcode.mode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.TestBotInit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.test.SkyStoneVuforia;

import java.util.List;
import static java.lang.Math.abs;


public abstract class AutonomousNew extends LinearOpMode {
    protected CompetitionBot robot;

    private ElapsedTime timer = new ElapsedTime();

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "SkyStone.tflite";
    private static final String LABEL_GOLD_MINERAL = "Stone";
    private static final String LABEL_SILVER_MINERAL = "Skystone";


    public void runTestLineDetect(Telemetry telemetry) throws InterruptedException {
        detectLineAndStop(false, 1800, telemetry, false);

    }

    private void detectLineAndStop(boolean isForward, int maxDist, Telemetry telemetry, boolean useDistanceSensor) throws InterruptedException {
        telemetry.addData("Initial reading: ", 0);
        telemetry.addData("Left  Level: ",  robot.LcolorSensor.blue());
        telemetry.addData("Right  Level: ",  robot.RcolorSensor.blue());
        telemetry.addData("Left Alpha  Level: ",  robot.LcolorSensor.alpha());
        telemetry.addData("Right Alpha  Level: ",  robot.RcolorSensor.alpha());
        telemetry.update();
        final int R_COLOR_THRESHOLD = 200;
        final int L_COLOR_THRESHOLD = 160;
        double RSpeed = .7;
        double LSpeed = .7;
        int direction = 1;

        if(!isForward) {
            direction = -1;
        }

        int initialPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int currentPosition = initialPosition;
        while(abs(currentPosition - initialPosition) < maxDist && (LSpeed != 0 || RSpeed != 0) && opModeIsActive()) {
            currentPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);

//            if(abs(currentPosition - initialPosition) > abs(.6 * (double) maxDist)) {
//                RSpeed = .2;
//                LSpeed = .2;
//            }

//            if(useDistanceSensor && robot.wallDistanceFront.getVoltage() < .12) { // .098 = 30 in
//                RSpeed = .2;
//                LSpeed = .2;
//            }

            if(robot.RcolorSensor.blue() > R_COLOR_THRESHOLD) {
                RSpeed = 0;
            }
            if(robot.LcolorSensor.blue() > L_COLOR_THRESHOLD) {
                LSpeed = 0;
            }
//            setMotors(direction * ramp(currentPosition, maxDist, LSpeed), direction * ramp(currentPosition, maxDist, LSpeed),
//                    direction * ramp(currentPosition, maxDist, RSpeed), direction * ramp(currentPosition, maxDist, RSpeed));
            setMotors(direction * LSpeed, direction * LSpeed, direction * RSpeed, direction * RSpeed);
        }
        setMotors(0,0,0,0);
    }

    public boolean underThreshold(int[] threshold) {
        if(robot.testColor.red() < threshold[0] & robot.testColor.green() < threshold[1]
                                                & robot.testColor.blue() < threshold[2]) {
            return true;
        } else {
            return false;
        }
    }

    public boolean overThreshold(int[] threshold) {
        if(robot.testColor.red() > threshold[0] & robot.testColor.green() > threshold[1]
                                                & robot.testColor.blue() > threshold[2]) {
            return true;
        } else {
            return false;
        }
    }

    public void detectBlockAndColor(boolean useSonar, Telemetry telemetry) throws InterruptedException {
        telemetry.addData("R: ", robot.testColor.red());
        telemetry.addData("G: ", robot.testColor.red());
        telemetry.addData("B: ", robot.testColor.red());
        telemetry.addData("Alpha: ", robot.testColor.alpha());
        telemetry.addData("Sonar V: ", robot.sonarDistance.getVoltage());
        final int[] YEllOW_THRESHOLD = {50, 30, 10};
        final int[] BLACK_THRESHOLD = {30, 30, 30};
        double LSpeed = .7;
        double RSpeed = .7;

        if(useSonar) {
            while(robot.sonarDistance.getVoltage() > 0.05) {
                setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
            }
        } else {
            while(underThreshold(YEllOW_THRESHOLD)) {
                setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
            }
        }

        LSpeed = .3;
        RSpeed = .3;
        while(overThreshold(BLACK_THRESHOLD)) {
            setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
        }
        setMotors(0, 0, 0, 0);

    }

    public void detectSkyStone() {
        SkyStoneVuforia vuforiaObj = new SkyStoneVuforia();
        vuforiaObj.runVuforia();
    }

    public void turnBy(double speed, double deltaAngle) throws InterruptedException {
        //
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double direction = diff > 0 ? 1 : -1;
        double adjustedSpeed;

        while (abs(diff) > .5 && opModeIsActive()) {
            // adjust speed when difference is smaller
            telemetry.addData("Gyro: ", robot.getPitch());
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? 0.1 : speed/2) : speed;
            robot.LFmotor.setPower(-direction * adjustedSpeed);
            robot.LBmotor.setPower(-direction * adjustedSpeed);
            robot.RFmotor.setPower(direction * adjustedSpeed);
            robot.RBmotor.setPower(direction * adjustedSpeed);
            currentAngle = robot.getPitch();
            diff = angleDiff(currentAngle, targetAngle);
            direction = diff > 0 ? 1 : -1;
            telemetry.addData("Diff: ", diff);
            telemetry.update();

        }
        setMotors(0,0,0,0);
    }

    public void turnUntil(double speed, double absAngle) throws InterruptedException {
        // enables us to use absolute angles to describe orientation of our robot
        double currentAngle = robot.getPitch();
        double diff = angleDiff(currentAngle, absAngle);
        turnBy(speed, diff);
    }

    private void rest(double seconds) {
        // delay function
        timer.reset();
        while(timer.time() < seconds && opModeIsActive()) {
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

    public void setMotors(double LF, double LB, double RF, double RB) throws InterruptedException{
        robot.LFmotor.setPower(LF);
        robot.LBmotor.setPower(LB);
        robot.RFmotor.setPower(RF);
        robot.RBmotor.setPower(RB);
    }

    public void forward(double speed, int deltaDistance) throws InterruptedException {
//        int linearDistance = // Conversion from enconder counts to linear distance

//        if (deltaDistance < 0) {
//            return;
//        }

        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
//        double currentDistance = robot.wallDistanceFront.getVoltage();

        double gyroCorrection;
//        while(distanceTarget == 0 ? avgPos < targetPos : currentDistance < distanceTarget && opModeIsActive()) {
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
//            currentDistance = robot.wallDistanceFront.getVoltage();
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
//            double rampedSpeed = distanceTarget == 0 ? ramp(avgPos, deltaDistance, targetPos, speed) : ramp(currentDistance, distanceTarget, speed);
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
            setMotors(clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection));

        }
        setMotors(0,0,0,0);
    }

    public void backward(double speed, int deltaDistance) throws InterruptedException {
//        if (deltaDistance < 0) {
//            return;
//        }

        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int targetPos = avgPos - deltaDistance;
        double targetPitch = robot.getPitch();
//        double currentDistance = robot.wallDistanceFront.getVoltage();

        double gyroCorrection;
//        while (distanceTarget == 0 ? avgPos > targetPos : currentDistance > distanceTarget && opModeIsActive()) {
        while (avgPos > targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
//            currentDistance = robot.wallDistanceFront.getVoltage();
//            telemetry.addData("WallDistance:", currentDistance);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
//            double rampedSpeed = distanceTarget == 0 ? ramp(avgPos, -deltaDistance, targetPos, speed) : ramp(currentDistance, distanceTarget, speed);
            double rampedSpeed = ramp(avgPos, -deltaDistance, targetPos, speed);
            setMotors(clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection));
            telemetry.update();

        }
        setMotors(0,0,0,0);
    }

    public void right(double speed, int deltaDistance) throws InterruptedException {
//        if (deltaDistance < 0) {
//            return;
//        }

        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);

        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
        double gyroCorrection;
        double distanceCorrection;
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
//            distanceCorrection = distanceReading != 0 ? (distanceReading - robot.wallDistanceFront.getVoltage()) * 10 : 0;
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
//            setMotors(clamp(rampedSpeed - gyroCorrection + distanceCorrection),
//                    clamp(-rampedSpeed - gyroCorrection + distanceCorrection),
//                    clamp(-rampedSpeed + gyroCorrection + distanceCorrection),
//                    clamp(rampedSpeed + gyroCorrection) + distanceCorrection);
            setMotors(clamp(rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection));
            telemetry.addData("PosL: ", avgPos);
            telemetry.update();

        }
        setMotors(0,0,0,0);
    }

    public void left(double speed, int deltaDistance) throws InterruptedException {
//        if (deltaDistance < 0) {
//            return;
//        }

        int avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);

        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
        double gyroCorrection;
        double distanceCorrection;
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
//            distanceCorrection = distanceReading != 0 ? (distanceReading - robot.wallDistanceFront.getVoltage()) * 10 : 0;
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
//            setMotors(clamp(-rampedSpeed - gyroCorrection + distanceCorrection),
//                    clamp(rampedSpeed - gyroCorrection + distanceCorrection),
//                    clamp(rampedSpeed + gyroCorrection + distanceCorrection),
//                    clamp(-rampedSpeed + gyroCorrection) + distanceCorrection);
            setMotors(clamp(-rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection));
            telemetry.addData("PosR: ", avgPos);
            telemetry.update();

        }
        setMotors(0,0,0,0);
    }

    // overloading allows us to define the following
//    public void forward(double speed, int deltaDistance) throws InterruptedException {
//        forward(speed, deltaDistance, 0);
//    }
//
//    public void backward(double speed, int deltaDistance) throws InterruptedException {
//        backward(speed, deltaDistance, 0);
//    }
//
//    public void right(double speed, int deltaDistance) throws InterruptedException {
//        right(speed, deltaDistance, 0);
//    }
//
//    public void left(double speed, int deltaDistance) throws InterruptedException {
//        left(speed, deltaDistance, 0);
//    }

    private double gyroCorrect(double targetPitch, double currentPitch) {
        double diff = angleDiff(currentPitch, targetPitch);
        if(abs(diff) < 1) {
            diff = 0;
        }
        return (diff * .03);
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

//    private void followGold() throws InterruptedException {
//        final double SEARCH_SPEED = .2;
//        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//        int goldMineralX = -1;
//        while(opModeIsActive() && updatedRecognitions.size() > 0) {
//            updatedRecognitions = tfod.getUpdatedRecognitions();
//            for(Recognition recognition : updatedRecognitions) {
//                if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                    goldMineralX = (int) recognition.getLeft();
//                }
//            }
//            double correction = (goldMineralX - 300) / 100;
//            setMotors(SEARCH_SPEED + correction, SEARCH_SPEED + correction, SEARCH_SPEED - correction, SEARCH_SPEED - correction);
//        }
//    }

    public boolean runTensorFlow(Telemetry telemetry, boolean isMineral) throws InterruptedException {
        boolean goldFound = false;
        boolean ballFound = false;
        final double SECONDS_TO_ABANDON = 2;
        timer.reset();
        while (!ballFound && !goldFound && timer.time() < SECONDS_TO_ABANDON) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if(updatedRecognitions.size() > 0) {
                    int goldMineralX = -1;
                    for(Recognition recognition : updatedRecognitions) {
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            goldFound = true;
                        }
                        if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                            ballFound = true;
                        }
                    }

                    if(!isMineral) {
                        if (goldFound) {
                            return true;
                        } else if (ballFound) {
                            return false;
                        }
                    } else {
                        if(ballFound) {
                            return false;
                        } else if (goldFound) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        final String VUFORIA_KEY = "AcRyVZf/////AAABmREbKz1DvE7yhFdOr9qQLSoq6MI/3Yoi9NEy+Z3poiBeamQswbGIX8ZqwRY5ElAoJe/4zqmdgZE73HPdbwsSDNk+9I17X4m8LGxRQaGOYsypI2HUvoFR+o141WvrzIYX2hhkANH7r+z5K0bY58wV6DUq3WCqN1fXWehixX956vv0wfXX2+YkVOo06U9llZwgmgE7gWKsgfcxmChr6PqXdiUtGsT4YztGG6Yr/c4Wlc6NDMIBgfmZWocJxl33oLpzO2DMkYWmgR3WOqsSBcjOEL2lvs5/D1UAVvuGe8uY6uMRjvZINIJznXnQbOJQrElTTT9G9mhjLR2ArCquvZbv/iCOh3k1DQMxsSkJXuyNAMle";

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CameraDevice.getInstance().setFlashTorchMode(true);
//        robot.Lights.setPower(.2);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    protected void activateTfod() {
        if(tfod != null) {
            tfod.activate();
        }
    }



}
