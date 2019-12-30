package org.firstinspires.ftc.teamcode.mode;

import android.sax.TextElementListener;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.test.SkyStoneVuforia;

import static java.lang.Math.abs;

// VUFORIA IMPORTS

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


public abstract class AutonomousNew extends LinearOpMode {
    protected CompetitionBot robot;

    private ElapsedTime timer = new ElapsedTime();

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = FRONT;
    private static final boolean PHONE_IS_PORTRAIT = false;


    private static final String VUFORIA_KEY =
            "AcRyVZf/////AAABmREbKz1DvE7yhFdOr9qQLSoq6MI/3Yoi9NEy+Z3poiBeamQswbGIX8ZqwRY5ElAoJe/4zqmdgZE73HPdbwsSDNk+9I17X4m8LGxRQaGOYsypI2HUvoFR+o141WvrzIYX2hhkANH7r+z5K0bY58wV6DUq3WCqN1fXWehixX956vv0wfXX2+YkVOo06U9llZwgmgE7gWKsgfcxmChr6PqXdiUtGsT4YztGG6Yr/c4Wlc6NDMIBgfmZWocJxl33oLpzO2DMkYWmgR3WOqsSBcjOEL2lvs5/D1UAVvuGe8uY6uMRjvZINIJznXnQbOJQrElTTT9G9mhjLR2ArCquvZbv/iCOh3k1DQMxsSkJXuyNAMle";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private int[] patternDistances = {200, 600, 1000};

    public void runBlueBlocksAuto(Telemetry telemetry) throws InterruptedException {
        double currentAngle = robot.getPitch();

        String pattern = detectSkyStone(true, telemetry);
        rightUntil(.5, 10, 1200);

        int patternDist = patternDistances[2];
        if (!pattern.equals("C") && !pattern.equals("None")) {
            if (pattern.equals("A")) {
                patternDist = patternDistances[0];
            } else if (pattern.equals("B")) {
                patternDist = patternDistances[1];
            }
            forward(.75, patternDist);
            turnBy(.75, 180);

            left(.75, 550);

            robot.IntakeMotor.setPower(1);
            forward(.75, 500);
            sleep(500);
            robot.IntakeMotor.setPower(0);

            right(.75, 550);
        } else { // Pattern C and default condition
            forward(.75, patternDist);
            turnBy(.75, -135);

            robot.IntakeMotor.setPower(1);
            forward(.5, 1250);
            sleep(500);
            robot.IntakeMotor.setPower(0);
            backward(.75, 1250);

            patternDist += 250;
        }

        turnUntil(.75, currentAngle + 180);

        backward(.75, 4850 - patternDist);

        turnUntil(.75,currentAngle + 90);

        backward(.5, 200);
        sleep(300);
        backward(.2,100);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        turnUntil(.5, currentAngle + 90);

        sleep(500);

        while (robot.sonarDistance.getVoltage() > .07) { robot.setMotors(.75, .75, .75, .75); }
        turnUntil(.5, currentAngle + 180);
        right(.5, 500);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);
        depositBlock(telemetry);
//        grabBlueFoundation();
//        rubBlueFoundationAuto()
/*        left(.75, 1750);
        backward(.5, 100);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);

        sleep(5000);

        forward(.75,100);
        right(.75,3000);
        backward(.75,100);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP); */

        if (!pattern.equals("A") && !pattern.equals("B")); {patternDist = patternDistances[2];}
        forward(.75,3050 - patternDist);

//        turnUntil(.75, currentAngle);

//        depositBlock(telemetry);

    }

    public void runRedBlocksAuto(Telemetry telemetry) throws InterruptedException {
        double currentAngle = robot.getPitch();

        String pattern = detectSkyStone(false, telemetry);
        rightUntil(.5, 10, 500);

        int patternDist = patternDistances[2];
        if (!pattern.equals("C") && !pattern.equals("None")) {
            if (pattern.equals("A")) {
                patternDist = patternDistances[0];
            } else if (pattern.equals("B")) {
                patternDist = patternDistances[1];
            }
            backward(.75, patternDist);

            right(.75, 500);

            robot.IntakeMotor.setPower(1);
            forward(.75, 500);
            sleep(500);
            robot.IntakeMotor.setPower(0);

            left(.75, 500);
        } else { // Pattern C and default condition
            backward(.75, patternDist);
            turnBy(.75, 45);

            robot.IntakeMotor.setPower(1);
            forward(.75, 1250);
            sleep(500);
            robot.IntakeMotor.setPower(0);
            backward(.75, 1250);
        }

        turnUntil(.75, currentAngle);

        backward(.75, 3000 - patternDist);

        // depositBlock(telemetry);

    }

    public void runBlueFoundationAuto(Telemetry telemetry) throws InterruptedException {
        while (robot.sonarDistance.getVoltage() < .125) { robot.setMotors(-1, -1, -1, -1); }
        turnUntil(.75, 0);

        backward(.5, 500);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        turnUntil(.5, 0);

        sleep(500);

        while (robot.sonarDistance.getVoltage() > .07) { robot.setMotors(.75, .75, .75, .75); }
        turnUntil(.5, 90);
        right(.5, 500);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        sleep(500);

        left(.75, 200);
        turnUntil(.5, 90);

        while (robot.sonarDistance.getVoltage() > .135) { robot.setMotors(.5, .5, .5, .5); }
        robot.setMotors(0, 0, 0, 0);
        turnUntil(.5, -90);

        sleep(500);

        runBlueBlocksAuto(telemetry);

    }

    public void depositBlock(Telemetry telemetry) throws InterruptedException {
        /*robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);

        double initPos = robot.SlideMotor.getCurrentPosition();

        telemetry.addData("Init Pos: ", initPos);

        while (Math.abs(robot.SlideMotor.getCurrentPosition() - initPos) < 100) { robot.SlideMotor.setPower(-.5); }
        robot.SlideMotor.setPower(0);

        telemetry.addData("Final Pos: ", initPos);

        sleep(3000);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);
        sleep(500);
        while (Math.abs(robot.SlideMotor.getCurrentPosition() - initPos) > 5) { robot.SlideMotor.setPower(-.5); }
        robot.SlideMotor.setPower(0);

        telemetry.update();*/

        robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);

        robot.SlideMotor.setPower(-.75);
        sleep(750);
        robot.SlideMotor.setPower(0);

        robot.armRotateServo.setPosition(CompetitionBot.ARM_OUT);

        /*robot.SlideMotor.setPower(.75);
        sleep(500);
        robot.SlideMotor.setPower(0);*/

        sleep(250);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);

        /*robot.SlideMotor.setPower(-.75);
        sleep(500);
        robot.SlideMotor.setPower(0);*/

        sleep(500);
        robot.armRotateServo.setPosition(CompetitionBot.ARM_IN);

        robot.SlideMotor.setPower(.75);
        sleep(600);
        robot.SlideMotor.setPower(0);
    }

    public void runTestLineDetect(Telemetry telemetry) throws InterruptedException {
        //detectLineAndStop(false, 1800, telemetry, false);

    }

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();
    }

    public String detectSkyStone(boolean isBlue, Telemetry telemetry) {
        String returnVal = "None";
        boolean patternFound = false;
        while (opModeIsActive()) {
            robot.setMotors(.1, -.1, -.1, .1);

            // check all the trackable targets to see which one (if any) is visible, while the measured distance > 5 cm
            // telemetry.addData("Initial Dist: ", robot.sensorRange.getDistance(DistanceUnit.CM));
            while (!isStopRequested() && robot.sensorRange.getDistance(DistanceUnit.CM) > 5) {
                // telemetry.addData("Dist: ", robot.sensorRange.getDistance(DistanceUnit.CM));
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && trackable.getName() == "Stone Target") {
                        telemetry.addLine("SkyStone found!");

                        if (!patternFound) {
                            robot.setMotors(0, 0, 0, 0);
                        }

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }

                        VectorF translation = lastLocation.getTranslation();

                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(1) / mmPerInch, translation.get(2) / mmPerInch, translation.get(0) / mmPerInch);

                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);


                        if (!patternFound) {
                            if (translation.get(1) / mmPerInch <= -3.0) {
                                returnVal = isBlue ? "C" : "A";
                                patternFound = true;
                            } else if (translation.get(1) / mmPerInch >= 3.0) {
                                returnVal = isBlue ? "A" : "C";
                                patternFound = true;
                            } else {
                                returnVal = "B";
                                patternFound = true;
                            }
                        }
                        telemetry.addData("Pattern: ", returnVal);
                        telemetry.update();

                        return returnVal;
                    }
                }
            }
        }
        telemetry.addLine("Not found!");
        telemetry.update();
        return returnVal;
    }

    public void detectLineAndGyro(boolean isForward, int maxDist, ColorSensor colorSensor, Telemetry telemetry) throws InterruptedException {
        final int BLUE_THRESHOLD = 200;
        final int RED_THRESHOLD = 200;

        int initialPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int currentPos = initialPos;

        int headingCorrection = robot.getPitch() > Math.abs(robot.getPitch() - 180) ? 0 : 180;

        while ((colorSensor.blue() < BLUE_THRESHOLD || colorSensor.red() < RED_THRESHOLD)
                && Math.abs(currentPos - initialPos) > maxDist && opModeIsActive()) {
            double power = isForward ? .75 : -.75;
            robot.setMotors(power, power, power, power);
        }

        turnUntil(.75, headingCorrection);
    }

/*
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
//            robot.setMotors(direction * ramp(currentPosition, maxDist, LSpeed), direction * ramp(currentPosition, maxDist, LSpeed),
//                    direction * ramp(currentPosition, maxDist, RSpeed), direction * ramp(currentPosition, maxDist, RSpeed));
            robot.setMotors(direction * LSpeed, direction * LSpeed, direction * RSpeed, direction * RSpeed);
        }
        robot.setMotors(0,0,0,0);
    }
*/

/*
    private void detectLineAndContinue(boolean isForward, Telemetry telemetry, boolean useDistanceSensor) throws InterruptedException {
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
        while(LSpeed != 0 || RSpeed != 0 && opModeIsActive()) {
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
//            robot.setMotors(direction * ramp(currentPosition, maxDist, LSpeed), direction * ramp(currentPosition, maxDist, LSpeed),
//                    direction * ramp(currentPosition, maxDist, RSpeed), direction * ramp(currentPosition, maxDist, RSpeed));
            robot.setMotors(direction * LSpeed, direction * LSpeed, direction * RSpeed, direction * RSpeed);
        }
    }
*/

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
                robot.setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
            }
        } else {
            while(underThreshold(YEllOW_THRESHOLD)) {
                robot.setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
            }
        }

        LSpeed = .3;
        RSpeed = .3;
        while(overThreshold(BLACK_THRESHOLD)) {
            robot.setMotors(LSpeed, LSpeed, RSpeed, RSpeed);
        }
        robot.setMotors(0, 0, 0, 0);

    }

/*    public void grabBlueFoundation(Telemetry telemetry) throws InterruptedException {
        backward(.5, 500);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        turnUntil(.5, 0);

        sleep(500);

        while (robot.sonarDistance.getVoltage() > .07) { robot.setMotors(.75, .75, .75, .75); }
        turnUntil(.5, 90);
        right(.5, 500);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);
    } */

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
        robot.setMotors(0,0,0,0);
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

//    public void setMotors(double LF, double LB, double RF, double RB) {
//        robot.LFmotor.setPower(LF);
//        robot.LBmotor.setPower(LB);
//        robot.RFmotor.setPower(RF);
//        robot.RBmotor.setPower(RB);
//    }

    public void forward(double speed, int deltaDistance) throws InterruptedException {
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
        double gyroCorrection;
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
            robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection));

        }
        robot.setMotors(0,0,0,0);
    }

    public void forwardUntil(double speed, int distance, int maxDist) throws InterruptedException {
        double targetPitch = robot.getPitch();
        double currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double gyroCorrection;
        while(currentDistance >= distance && abs(currentPos - initPos) < maxDist && opModeIsActive()) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);
            robot.setMotors(clamp(speed - gyroCorrection),
                    clamp(speed - gyroCorrection),
                    clamp(speed + gyroCorrection),
                    clamp(speed + gyroCorrection));

            telemetry.addData("dist: ", currentDistance);
            telemetry.update();
        }
        robot.setMotors(0,0,0,0);
    }

    public void backward(double speed, int deltaDistance) throws InterruptedException {
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int targetPos = avgPos - deltaDistance;
        double targetPitch = robot.getPitch();

        double gyroCorrection;
        while (avgPos > targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            double rampedSpeed = ramp(avgPos, -deltaDistance, targetPos, speed);
            robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection));
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void backUntil(double speed, int distance, int maxDist) throws InterruptedException {
        double targetPitch = robot.getPitch();
        double currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double gyroCorrection;
        while(currentDistance >= distance && abs(currentPos - initPos) < maxDist && opModeIsActive()) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);
            robot.setMotors(clamp(-speed - gyroCorrection),
                    clamp(-speed - gyroCorrection),
                    clamp(-speed + gyroCorrection),
                    clamp(-speed + gyroCorrection));

            telemetry.addData("dist: ", currentDistance);
            telemetry.update();
        }
        robot.setMotors(0,0,0,0);
    }

    public void right(double speed, int deltaDistance) throws InterruptedException {
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);

        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
        double gyroCorrection;
        double distanceCorrection;
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
            robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed - gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection));
            telemetry.addData("PosL: ", avgPos);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void rightUntil(double speed, int distance, int maxDist) throws InterruptedException {
        double targetPitch = robot.getPitch();
        double currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double gyroCorrection;
        while(currentDistance >= distance && abs(currentPos - initPos) < maxDist && opModeIsActive()) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);
            robot.setMotors(clamp(speed - gyroCorrection),
                    clamp(-speed - gyroCorrection),
                    clamp(-speed + gyroCorrection),
                    clamp(speed + gyroCorrection));

        telemetry.addData("dist: ", currentDistance);
        telemetry.update();
        }
        robot.setMotors(0,0,0,0);
    }

    public void left(double speed, int deltaDistance) throws InterruptedException {
        int avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);

        int targetPos = avgPos + deltaDistance;
        double targetPitch = robot.getPitch();
        double gyroCorrection;
        double distanceCorrection;
        while(avgPos < targetPos && opModeIsActive()) {
            avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            double rampedSpeed = ramp(avgPos, deltaDistance, targetPos, speed);
            robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed - gyroCorrection),
                    clamp(rampedSpeed + gyroCorrection),
                    clamp(-rampedSpeed + gyroCorrection));
            telemetry.addData("PosR: ", avgPos);
            telemetry.update();

        }
        robot.setMotors(0,0,0,0);
    }

    public void leftUntil(double speed, int distance, int maxDist) throws InterruptedException {
        double targetPitch = robot.getPitch();
        double currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);

        int initPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double gyroCorrection;
        while(currentDistance >= distance && abs(currentPos - initPos) < maxDist && opModeIsActive()) {
            currentPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
            currentDistance = robot.sensorRange.getDistance(DistanceUnit.CM);
            robot.setMotors(clamp(-speed - gyroCorrection),
                    clamp(speed - gyroCorrection),
                    clamp(speed + gyroCorrection),
                    clamp(-speed + gyroCorrection));

            telemetry.addData("dist: ", currentDistance);
            telemetry.update();
        }
        robot.setMotors(0,0,0,0);
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

}
