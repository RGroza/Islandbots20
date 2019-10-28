package org.firstinspires.ftc.teamcode.mode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


public abstract class AutonomousNew extends LinearOpMode {
    protected CompetitionBot robot;

    private ElapsedTime timer = new ElapsedTime();

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
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

    public void setMotors(double LF, double LB, double RF, double RB) {
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

    public void runVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
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

    public boolean detectSkyStone() {
        while (!isStopRequested()) {
            setMotors(0.5, 0.5, 0.5, 0.5);

            // check all the trackable targets to see which one (if any) is visible, while the measured distance > 5 cm
            while (robot.sensorRange.getDistance(DistanceUnit.CM) > 5) {
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && trackable.getName() == "Stone Target") {
                        telemetry.addLine("SkyStone Found!");
                        setMotors(0, 0, 0, 0);
                        return true;
                    }
                }
            }
        }
        return false;
    }
}
