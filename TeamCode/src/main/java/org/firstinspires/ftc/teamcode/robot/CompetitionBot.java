package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
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
    public AnalogInput sonarDistance;
    public DistanceSensor frontDistance, backDistance;
    public NormalizedColorSensor LcolorSensor, RcolorSensor;

    // motors
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor, SlideMotor, IntakeMotor, TapeMeasure, LEDPower;
    public Servo grabberServo, armRotateServo, capStoneServo, Lfoundation, Rfoundation, beamsServo;

    // Servo constants
    public static final double GRABBER_OPEN = .7;
    public static final double GRABBER_CLOSED = .3;

    public static final double ARM_OUT = .61;
    public static final double ARM_IN = .25;

    public static final double FOUND_LEFT = .8;
    public static final double FOUND_DOWN = .5;
    public static final double FOUND_RIGHT = .2;

    public static final double CAPSTONE_OPEN = .6;
    public static final double CAPSTONE_CLOSED = .2;

    public static final double BEAMS_UP = .6;
    public static final double BEAMS_DOWN = .2;

    public static final double MAX_SPEED = .5;

    public static final double DRIVETAIN_RPM = 1150;
    public static final double DRIVETAIN_PPR = 146;

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
        LEDPower = hwMap.dcMotor.get("LEDPower");
        TapeMeasure = hwMap.dcMotor.get("TapeMeasure");

        // gyro hwMap
        gyro = hwMap.get(BNO055IMU.class, "gyro");

        // color sensors
        LcolorSensor = hwMap.get(NormalizedColorSensor.class, "LcolorSensor");
        RcolorSensor = hwMap.get(NormalizedColorSensor.class, "RcolorSensor");

        // analog sensors
        sonarDistance = hwMap.analogInput.get("sonarDistance");

        // distance sensors
        frontDistance = hwMap.get(DistanceSensor.class, "frontDistance");
        backDistance = hwMap.get(DistanceSensor.class, "backDistance");

        // motor encoders init
        resetEncoders();

        // motor directions
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        // servos
        grabberServo = hwMap.servo.get("grabberServo");
        armRotateServo = hwMap.servo.get("armRotateServo");
        capStoneServo = hwMap.servo.get("capStoneServo");
        Lfoundation = hwMap.servo.get("Lfoundation");
        Rfoundation = hwMap.servo.get("Rfoundation");
        beamsServo = hwMap.servo.get("beamsServo");
        // TODO: move servos to initial positions

        Lfoundation.setPosition(CompetitionBot.FOUND_RIGHT);
        Rfoundation.setPosition(CompetitionBot.FOUND_LEFT);
        grabberServo.setPosition(CompetitionBot.GRABBER_OPEN);
        beamsServo.setPosition(CompetitionBot.BEAMS_UP);

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

    public double[] mecanumMove(double joystickX, double joystickY, double rotation, boolean slowToggle, Telemetry telemetry) {
        double SPEED_REDUCTION;

        if (slowToggle){
            SPEED_REDUCTION = .5*MAX_SPEED;
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
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double SingleClamp(double power) {
        // ensures power does not exceed abs(1)
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
