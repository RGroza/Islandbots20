package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class CompetitionBot {
    // sensors
    public BNO055IMU gyro;
    public NormalizedColorSensor LcolorSensor, RcolorSensor;

    // motors
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor, SlideMotor, IntakeMotor, LFlywheelMotor, RFlywheelMotor;
    public Servo grabberServo, armRotateServo, ringFeedServo;

    // Servo constants
    public static final double GRABBER_OPEN = .15;
    public static final double GRABBER_CLOSED = .5;

    public static final double FEED_OPEN = .46;
    public static final double FEED_CLOSED = .29;

    public static final double ARM_OUT = .05;
    public static final double ARM_IN = .6;
    public static final double ARM_MID = .35;
    public static final double ARM_AUTO = .25;

    public static final double MAX_SPEED = .7;

    public static final double DRIVETAIN_RPM = 1150;
    public static final double DRIVETAIN_PPR = 146;

    public static final double HIGH_GOAL_SPEED = .65;
    public static final double POWER_SHOT_SPEED = .65;
    public static final double FLYWHEEL_SPEED_DIFF = .2;


    public CompetitionBot(HardwareMap hwMap, Telemetry telemetry) {
        // gyro initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        // motors
        RFmotor = hwMap.dcMotor.get("RFmotor");
        RBmotor = hwMap.dcMotor.get("RBmotor");
        LFmotor = hwMap.dcMotor.get("LFmotor");
        LBmotor = hwMap.dcMotor.get("LBmotor");
        SlideMotor = hwMap.dcMotor.get("SlideMotor");
        IntakeMotor = hwMap.dcMotor.get("IntakeMotor");
        LFlywheelMotor = hwMap.dcMotor.get("LFlywheelMotor");
        RFlywheelMotor = hwMap.dcMotor.get("RFlywheelMotor");
        RFlywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // gyro hwMap
        gyro = hwMap.get(BNO055IMU.class, "gyro");

        // color sensors
//        LcolorSensor = hwMap.get(NormalizedColorSensor.class, "LcolorSensor");
        RcolorSensor = hwMap.get(NormalizedColorSensor.class, "RcolorSensor");

        // motor encoders init
        resetEncoders();

        // motor directions
//        RFmotor.setDirection(DcMotor.Direction.REVERSE);
//        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        // servos
        grabberServo = hwMap.servo.get("grabberServo");
        armRotateServo = hwMap.servo.get("armRotateServo");
        ringFeedServo = hwMap.servo.get("ringFeedServo");
        grabberServo.setPosition(CompetitionBot.GRABBER_CLOSED);
        armRotateServo.setPosition(CompetitionBot.ARM_IN);
        ringFeedServo.setPosition(CompetitionBot.FEED_CLOSED);

        // gyro
        gyro.initialize(parameters);

        telemetry.addData("Successfully Initialized", null);
        telemetry.update();
    }

    // joystickY and joystickX expected to be within [-1,1]
    private double getVAngle(double joystickX, double joystickY) {
        if(joystickX == 0 && joystickY == 0) {
            return 0;
        } else {
            // transform angle for movement
            double angle = Math.atan2(joystickY, joystickX);
            if (Math.abs(angle) < .2) {
                return -Math.PI/4;
            } else if(Math.abs(Math.abs(angle) - Math.PI) < .2) {
                return 3 * Math.PI/4;
            } else {
                return angle - Math.PI/4;
            }
        }
    }

    private double getVMagnitude(double joystickX, double joystickY) {
        double Magnitude;
        Magnitude = Math.hypot(joystickX, joystickY);
        if(Magnitude < .1) {
            Magnitude = 0;
        }
        return Magnitude;
    }

    public double[] mecanumMove(double joystickX, double joystickY, double rotation, boolean slowToggle, boolean fastHold, Telemetry telemetry) {
        double SPEED_REDUCTION;

        if (slowToggle){
            SPEED_REDUCTION = .5*MAX_SPEED;
        } else if (fastHold) {
            SPEED_REDUCTION = 1;
        } else {
            SPEED_REDUCTION = MAX_SPEED;
        }

        double vMagnitude = getVMagnitude(joystickX, joystickY);
        double vAngle = getVAngle(joystickX, joystickY);

        vMagnitude *= vMagnitude; // Non-linear magnitude input

        double LB = vMagnitude * Math.cos(vAngle) + (.7 * rotation);
        double RB = vMagnitude * Math.sin(vAngle) - (.7 * rotation);
        double LF = vMagnitude * Math.sin(vAngle) + (.7 * rotation);
        double RF = vMagnitude * Math.cos(vAngle) - (.7 * rotation);

        LF *= SPEED_REDUCTION;
        LB *= SPEED_REDUCTION;
        RF *= SPEED_REDUCTION;
        RB *= SPEED_REDUCTION;

        double[] powerVals = {LF, LB, RF, RB};

        LF *= clamp(powerVals);
        LB *= clamp(powerVals);
        RF *= clamp(powerVals);
        RB *= clamp(powerVals);

        setMotors(LF, LB, RF, RB);

        return powerVals;
    }

    public void setMotors(double LF, double LB, double RF, double RB) {
        LFmotor.setPower(LF);
        LBmotor.setPower(LB);
        RFmotor.setPower(RF);
        RBmotor.setPower(RB);
    }

    public void setMotorRPM(int LF, int LB, int RF, int RB) {
        LFmotor.setPower(LF/DRIVETAIN_RPM);
        LBmotor.setPower(LB/DRIVETAIN_RPM);
        RFmotor.setPower(RF/DRIVETAIN_RPM);
        RBmotor.setPower(RB/DRIVETAIN_RPM);
    }

    public double getPitch() {
        Orientation angOrientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angOrientation.firstAngle;
    }

    public Orientation getOrientation() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFlywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFlywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFlywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double clamp(double power) {
        if (power > 1) {
            return 1;
        }
        if (power < -1) {
            return -1;
        }
        return power;
    }

    private double clamp(double[] powerVals) {
        double max = Math.max(Math.max(powerVals[0], powerVals[1]), Math.max(powerVals[2], powerVals[3]));
        if (max > 1) {
            return 1 / max;
        } else {
            return 1;
        }
    }

}
