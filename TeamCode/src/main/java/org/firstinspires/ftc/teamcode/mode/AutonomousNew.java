package org.firstinspires.ftc.teamcode.mode;

import android.sax.TextElementListener;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.CompetitionBot;
import org.firstinspires.ftc.teamcode.robot.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    private double[] patternVoltages = {.16, .20, .25};
    private double[] patternDistances = {.5, 1.25, 2.25};

    private double gyroCorrectConst = .02;

    private PIDController PIDControl = new PIDController(.01, .075, .15);

    public void initPIDControl(Telemetry telemetry, PIDController PID) {
        PID.setOutputLimits(-.05, .05);
        PID.setSetpoint(0);
    }

    public void runBlueBlocksAuto(Telemetry telemetry) throws InterruptedException {
        double currentAngle = robot.getPitch();

        String pattern = detectSkyStone(true, telemetry);

        telemetry.addData("Pattern: ", pattern);
        telemetry.update();

        rightUntil(.3, 25, 5, true);
        turnUntil(.5, currentAngle + 180);

        double patternDist = patternDistances[2];
        if (!pattern.equals("C") && !pattern.equals("None")) {
            if (pattern.equals("A")) {
                patternDist = patternDistances[0];
            } else if (pattern.equals("B")) {
                patternDist = patternDistances[1];
            }
            backward(.3, patternDist, true);
            turnUntil(.5, currentAngle + 180);

            left(.3, 2.25, true);
            turnUntil(.5, currentAngle + 180);

            robot.IntakeMotor.setPower(1);
            forward(.3, 1.5, true);
            sleep(500);
            robot.IntakeMotor.setPower(0);

            right(.3, 2.5, true);
        } else { // Pattern C and default condition
            turnUntil(.5, currentAngle - 180);
            backward(.3, patternDist, true);
            turnUntil(.5, currentAngle - 135);

            robot.IntakeMotor.setPower(1);
            forward(.3, 4, true);
            sleep(500);
            robot.IntakeMotor.setPower(0);
            backward(.3, 5, true);
        }

        // Park after BlockAuto
        turnUntil(.5, currentAngle + 180);
        backward(.3, 6, true);

        forward(.3, 3, true);

        // Transition to FoundationAuto
//        turnUntil(.5, currentAngle + 180);
//        backward(.3, 11 - patternDist, true);
//
//        turnUntil(.5, currentAngle + 90);
//
//        grabBlueFoundation(telemetry);
//        depositBlock(telemetry);
//
//        forward(.3, 4, true);

    }

    public void runRedBlocksAuto(Telemetry telemetry) throws InterruptedException {
        double currentAngle = robot.getPitch();

        String pattern = detectSkyStone(true, telemetry);

        telemetry.addData("Pattern: ", pattern);
        telemetry.update();

        rightUntil(.3, 25, 5, true);
        turnUntil(.5, currentAngle);

        double patternDist = patternDistances[2];
        if (!pattern.equals("C") && !pattern.equals("None")) {
            if (pattern.equals("A")) {
                patternDist = patternDistances[0];
            } else if (pattern.equals("B")) {
                patternDist = patternDistances[1];
            }
            forward(.3, patternDist, true);
            turnUntil(.5, currentAngle);

            right(.25, 2.5, true);
            turnUntil(.5, currentAngle);

            robot.IntakeMotor.setPower(1);
            forward(.3, 1.5, true);
            sleep(500);
            robot.IntakeMotor.setPower(0);

            left(.25, 2.5, true);
        } else { // Pattern C and default condition
            turnUntil(.5, currentAngle - 180);
            forward(.3, patternDist, true);
            turnUntil(.5, currentAngle - 45);

            robot.IntakeMotor.setPower(1);
            forward(.3, 4, true);
            sleep(500);
            robot.IntakeMotor.setPower(0);
            backward(.3, 5, true);
        }

        turnUntil(.5, currentAngle);

        backward(.3, 11 - patternDist, true);

        turnUntil(.5, currentAngle - 90);

        grabRedFoundation(telemetry);
        depositBlock(telemetry);

        forward(.3, 4, true);
    }

    public void runBlueFoundationAuto(Telemetry telemetry) throws InterruptedException {
        right(.3, 1, true);
        grabBlueFoundation(telemetry);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        forward(.3, 5, true);

//        moveUntilSonar(.4, .135, 15, true);
//        turnUntil(.4, currentAngle);
//
//        sleep(500);
//
//        runBlueBlocksAuto(telemetry);
    }

    public void runRedFoundationAuto(Telemetry telemetry) throws InterruptedException {
        left(.3, 1, true);
        grabRedFoundation(telemetry);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        forward(.3, 5, true);

//        moveUntilSonar(.4, .135, 15, true);
//        turnUntil(.4, currentAngle);
//
//        sleep(500);
//
//        runRedBlocksAuto(telemetry);
    }

    public void grabBlueFoundation(Telemetry telemetry) throws InterruptedException {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        moveUntilLaser(.3, 3.0, 10, true); // using backDistance
        turnUntil(.5, currentAngle);

        backward(.25, .75, true);

        turnUntil(.5, currentAngle);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        turnUntil(.5, currentAngle);

        forward(.3, 3, true);
        turnUntil(.35, currentAngle + 90);
        right(.3, 1, true);
        turnUntil(.35, currentAngle + 90);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        sleep(500);

        right(.3, 2, true);
        turnUntil(.5, currentAngle + 90);
    }

    public void grabRedFoundation(Telemetry telemetry) throws InterruptedException {
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        double currentAngle = robot.getPitch();

        turnUntil(.5, currentAngle);

        moveUntilLaser(.3, 3.0, 10, true); // using backDistance
        turnUntil(.5, currentAngle);

        backward(.25, .75, true);

        turnUntil(.5, currentAngle);
        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_DOWN);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_DOWN);
        sleep(500);

        turnUntil(.5, currentAngle);

        forward(.3, 3, true);
        turnUntil(.35, currentAngle - 90);
        left(.3, 1, true);
        turnUntil(.35, currentAngle - 90);

        robot.Lfoundation.setPosition(CompetitionBot.L_FOUND_UP);
        robot.Rfoundation.setPosition(CompetitionBot.R_FOUND_UP);

        sleep(500);

        left(.3, 2, true);
        turnUntil(.4, currentAngle - 90);
    }

    public void depositBlock(Telemetry telemetry) throws InterruptedException {
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);

        robot.SlideMotor.setPower(-.75);
        sleep(750);
        robot.SlideMotor.setPower(0);

        robot.armRotateServo.setPosition(CompetitionBot.ARM_OUT);

        sleep(250);
        robot.grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);

        sleep(500);
        robot.armRotateServo.setPosition(CompetitionBot.ARM_IN);

        robot.SlideMotor.setPower(.75);
        sleep(600);
        robot.SlideMotor.setPower(0);
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

        robot.LEDPower.setPower(1);
    }

    public String detectSkyStone(boolean isBlue, Telemetry telemetry) throws InterruptedException {
        String returnVal = "None";
        boolean patternFound = false;

        initPIDControl(telemetry, PIDControl);
        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        double maxDist = 1000;
        double currentPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        double initPos = currentPos;

        double currentTime = System.currentTimeMillis();
        double initTime = currentTime;

        // check all the trackable targets to see which one (if any) is visible, while the measured distance > 5 cm and timeout after 4s
        while (opModeIsActive() && robot.sideDistance.getDistance(DistanceUnit.CM) > 25 && abs(currentPos - initPos) < maxDist) {// && currentTime - initTime < 5000) {
            currentPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            currentTime = System.currentTimeMillis();

            actualPitch = robot.getPitch();
            output = PIDControl.getOutput(actualPitch, targetPitch);
            robot.setMotors(clamp(.225 - output), clamp(-.225 - output),
                            clamp(-.225 + output), clamp(.225 + output));

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

                    robot.LEDPower.setPower(0);

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
        robot.setMotors(0, 0, 0 ,0);
        telemetry.addLine("Not found!");
        telemetry.update();
        sleep(500);
        return returnVal;
    }

    public void detectLineAndGyro(boolean isForward, int maxDist, ColorSensor colorSensor, Telemetry telemetry) throws InterruptedException {
        final int BLUE_THRESHOLD = 200;
        final int RED_THRESHOLD = 200;

        int initialPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int currentPos = initialPos;

        int headingCorrection = robot.getPitch() > Math.abs(robot.getPitch() - 180) ? 0 : 180;

        while (opModeIsActive() && (colorSensor.blue() < BLUE_THRESHOLD || colorSensor.red() < RED_THRESHOLD)
                && Math.abs(currentPos - initialPos) > maxDist && opModeIsActive()) {
            double power = isForward ? .3 : -.3;
            robot.setMotors(power, power, power, power);
        }

        turnUntil(.3, headingCorrection);
    }

    public void sonarPatternTest(Telemetry telemetry) throws InterruptedException {
        moveUntilSonar(.25, patternVoltages[0], 10, true);
        turnUntil(.4, 0);
        sleep(1000);
        moveUntilSonar(.25, patternVoltages[1], 10, true);
        turnUntil(.4, 0);
        sleep(1000);
        moveUntilSonar(.25, patternVoltages[2], 10, true);
        turnUntil(.4, 0);
        sleep(1000);
        moveUntilSonar(.25, .45, 10, true);
        turnUntil(.4, 0);
    }

    public void encoderPatternTest(Telemetry telemetry) throws InterruptedException {
        backward(.3, patternDistances[0], true);
        turnUntil(.4, 0);
        sleep(1000);
        backward(.3, patternDistances[1]-patternDistances[0], true);
        turnUntil(.4, 0);
        sleep(1000);
        backward(.3, patternDistances[2]-patternDistances[1], true);
        turnUntil(.4, 0);
        sleep(1000);
    }

    public void detectLineAndStop(boolean isForward, double speed, int maxDist, Telemetry telemetry) throws InterruptedException {
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

        int initialPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int currentPosition = initialPosition;
        int maxPos = initialPosition + maxSteps;
        while(opModeIsActive() && abs(currentPosition - initialPosition) < maxPos && (LSpeed != 0 || RSpeed != 0)) {
            currentPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            colorsL = robot.LcolorSensor.getNormalizedColors();
            colorsR = robot.RcolorSensor.getNormalizedColors();

            if (colorsL.blue > L_COLOR_THRESHOLD || colorsL.red > L_COLOR_THRESHOLD) {
                LSpeed = 0;
            }
            if (colorsR.blue > R_COLOR_THRESHOLD || colorsR.red > R_COLOR_THRESHOLD) {
                RSpeed = 0;
            }

            robot.setMotors(dir*LSpeed, dir*LSpeed, dir*RSpeed, dir*RSpeed);
        }
        robot.setMotors(0,0,0,0);
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

        int initialPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int currentPosition = initialPosition;
        int maxPos = initialPosition + maxSteps;
        while(opModeIsActive() && abs(currentPosition - initialPosition) < maxPos && (LSpeed != 0 || RSpeed != 0)) {
            currentPosition = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            colorsL = robot.LcolorSensor.getNormalizedColors();
            colorsR = robot.RcolorSensor.getNormalizedColors();

            if (colorsL.blue > L_COLOR_THRESHOLD || colorsL.red > L_COLOR_THRESHOLD) {
                LSpeed = 0;
            }
            if (colorsR.blue > R_COLOR_THRESHOLD || colorsR.red > R_COLOR_THRESHOLD) {
                RSpeed = 0;
            }

            robot.setMotors(dir*LSpeed, dir*LSpeed, dir*RSpeed, dir*RSpeed);
        }
    }

    public void turnBy(double speed, double deltaAngle) throws InterruptedException {
        //
        double currentAngle = robot.getPitch();
        double targetAngle = (currentAngle + deltaAngle) % 360;
        double diff = angleDiff(currentAngle, targetAngle);
        double direction = diff > 0 ? 1 : -1;
        double adjustedSpeed;

        while (opModeIsActive() && abs(diff) > .5) {
            // adjust speed when difference is smaller
            telemetry.addData("Gyro: ", robot.getPitch());
            adjustedSpeed = abs(diff) < 30 ? (abs(diff) < 10 ? 0.2 : speed/2) : speed;
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
        double rampRange = .1*(targetVal-initVal);
        if (abs(currentVal-initVal) <= rampRange) {
            if (linearRamp) {
                return ((currentVal-initVal) / rampRange) * (speed-minSpeed) + minSpeed;
            } else {
                return ((currentVal-initVal) / rampRange) * (speed-minSpeed)*(speed-minSpeed) + minSpeed;
            }
        } else if (abs(targetVal-currentVal) <= rampRange) {
            if (linearRamp) {
                return (targetVal-currentVal) / rampRange * (speed-minSpeed) + minSpeed;
            } else {
                return (targetVal-currentVal) / rampRange * (speed-minSpeed)*(speed-minSpeed) + minSpeed;
            }
        }
        return speed;
    }

    private void stopMotors(int durationTime) {
        robot.LFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.LFmotor.setTargetPosition(robot.LFmotor.getCurrentPosition());
        robot.LBmotor.setTargetPosition(robot.LBmotor.getCurrentPosition());
        robot.RFmotor.setTargetPosition(robot.RFmotor.getCurrentPosition());
        robot.RBmotor.setTargetPosition(robot.RBmotor.getCurrentPosition());

        robot.setMotors(.5, .5, .5, .5);

        sleep(durationTime);

        robot.LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveUntilLaser(double speed, double distance, double maxRevs, boolean PID) throws InterruptedException {
        if (distance - robot.backDistance.getDistance(DistanceUnit.CM) > 0) {
            forwardUntilLaser(speed, distance, maxRevs, PID);
        } else {
            backwardUntilLaser(speed, distance, maxRevs, PID);
        }
    }

    public void moveUntilSonar(double speed, double voltage, double maxRevs, boolean PID) throws InterruptedException {
        if (voltage - robot.sonarDistance.getVoltage() > 0) {
            backwardUntilSonar(speed, voltage, maxRevs, PID);
        } else {
            forwardUntilSonar(speed, voltage, maxRevs, PID);
        }
    }

    public void forward(double speed, double revCount, boolean PID) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int initPos = avgPos;

        int targetPos = avgPos + stepCount;
        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            double rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                                clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void forwardUntilLaser(double speed, double distance, double maxRevs, boolean PID) throws InterruptedException {
        double currentVal = robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && currentVal < targetVal && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = robot.backDistance.getDistance(DistanceUnit.CM);

            double rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                                clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void forwardUntilSonar(double speed, double voltage, double maxRevs, boolean PID) throws InterruptedException {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && currentVal > targetVal && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .05 ? robot.sonarDistance.getVoltage() : currentVal;

            double rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(rampedSpeed - output),
                        clamp(rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backward(double speed, double revCount, boolean PID) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
        int initPos = avgPos;

        int targetPos = avgPos - stepCount;
        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && avgPos > targetPos) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 4.0);
            double rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                                clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backwardUntilLaser(double speed, double distance, double maxRevs, boolean PID) throws InterruptedException {
        double currentVal = robot.backDistance.getDistance(DistanceUnit.CM);
        double targetVal = distance;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && currentVal > targetVal && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentVal = robot.backDistance.getDistance(DistanceUnit.CM);

            double rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                        clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void backwardUntilSonar(double speed, double voltage, double maxRevs, boolean PID) throws InterruptedException {
        double currentVal = robot.sonarDistance.getVoltage();
        double targetVal = voltage;
        double initVal = currentVal;

        int maxSteps = (int) (maxRevs*CompetitionBot.DRIVETAIN_PPR);

        int initPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int currentPos = initPos;

        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && currentVal < targetVal && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            // Condition used to eliminate noise
            currentVal = robot.sonarDistance.getVoltage() > .075 ? robot.sonarDistance.getVoltage() : currentVal;

            double rampedSpeed = rampSpeed(currentVal, initVal, targetVal, speed, .1, false);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(-rampedSpeed - output),
                        clamp(-rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void left(double speed, double revCount, boolean PID) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
        int initPos = avgPos;

        int targetPos = avgPos + stepCount;
        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            double rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .2, true);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-rampedSpeed - output), clamp(rampedSpeed - output),
                                clamp(rampedSpeed + output), clamp(-rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(-rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed - gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

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

        double gyroCorrection;
        while (opModeIsActive() && currentDistance >= distance && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.RFmotor.getCurrentPosition() + robot.LBmotor.getCurrentPosition()) / 2.0);
            currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(-speed - output), clamp(speed - output),
                                clamp(speed + output), clamp(-speed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(-speed - gyroCorrection),
                        clamp(speed - gyroCorrection),
                        clamp(speed + gyroCorrection),
                        clamp(-speed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

    public void right(double speed, double revCount, boolean PID) throws InterruptedException {
        int stepCount = (int) (revCount*CompetitionBot.DRIVETAIN_PPR);
        int avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
        int initPos = avgPos;

        int targetPos = avgPos + stepCount;
        double actualPitch = robot.getPitch();
        double targetPitch = actualPitch;
        double output;

        if (PID) {
            initPIDControl(telemetry, PIDControl);
        }

        double gyroCorrection;
        while (opModeIsActive() && avgPos < targetPos) {
            avgPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            double rampedSpeed = rampSpeed(avgPos, initPos, targetPos, speed, .2, true);

            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(rampedSpeed - output), clamp(-rampedSpeed - output),
                        clamp(-rampedSpeed + output), clamp(rampedSpeed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed - gyroCorrection),
                        clamp(-rampedSpeed + gyroCorrection),
                        clamp(rampedSpeed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

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

        double gyroCorrection;
        while (opModeIsActive() && currentDistance >= distance && abs(currentPos - initPos) < maxSteps) {
            currentPos = (int) ((robot.LFmotor.getCurrentPosition() + robot.RBmotor.getCurrentPosition()) / 2.0);
            currentDistance = robot.sideDistance.getDistance(DistanceUnit.CM);
            if (PID) {
                actualPitch = robot.getPitch();
                output = PIDControl.getOutput(actualPitch, targetPitch);
                robot.setMotors(clamp(speed - output), clamp(-speed - output),
                                clamp(-speed + output), clamp(speed + output));
            } else {
                gyroCorrection = gyroCorrect(targetPitch, robot.getPitch());
                robot.setMotors(clamp(speed - gyroCorrection),
                        clamp(-speed - gyroCorrection),
                        clamp(-speed + gyroCorrection),
                        clamp(speed + gyroCorrection));
            }
        }
        robot.setMotors(0,0,0,0);
    }

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
