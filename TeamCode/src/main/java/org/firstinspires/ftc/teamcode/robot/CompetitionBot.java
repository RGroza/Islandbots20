package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by KaDon on 8/26/2018.
 */

public class CompetitionBot {
    public BNO055IMU gyro;
    public AnalogInput sonarDistance;
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor;
    public I2cDeviceSynch opticalDistance;
    //, winchLift, mineralLift, SweeperArm, Lights;
    //public Servo SweeperBoxPivot, SweeperSlide;
    //public CRServo SweeperMotor;
    //public I2cDeviceSynch pixyCam;

    boolean slowMove;

    // Servo constants
    public final static double SWEEPER_UP_POS = .75;
    public final static double SWEEPER_DOWN_POS = .25;
    public final static double SWEEPER_HALF_POS = .5;
    // public ColorSensor RcolorSensor;
    // public ColorSensor LcolorSensor;

    public ColorSensor testColor, LcolorSensor, RcolorSensor;

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
//        SweeperArm = hwMap.dcMotor.get("SweeperArm");
//        winchLift = hwMap.dcMotor.get("WinchLift");
//        mineralLift = hwMap.dcMotor.get("MineralLift");
//        Lights = hwMap.dcMotor.get("Lights");

        // servos
        gyro = hwMap.get(BNO055IMU.class, "gyro");
//        SweeperBoxPivot = hwMap.servo.get("SweeperBoxPivot");
//        SweeperSlide = hwMap.servo.get("SweeperSlide");

        // CRServos
//        SweeperMotor = hwMap.crservo.get("SweeperMotor");

        // Color sensors
        testColor = hwMap.colorSensor.get("testColor");
        LcolorSensor = hwMap.colorSensor.get("LcolorSensor");
        RcolorSensor = hwMap.colorSensor.get("RcolorSensor");

        sonarDistance = hwMap.analogInput.get("sonarDistance");

        // motor encoders init
        resetEncoders();

        // motor directions
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        gyro.initialize(parameters);
        telemetry.addData("Successfully Initialized", null);
        telemetry.update();
//        slowMove = false;
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
        if(Magnitude < .3) {
            Magnitude = 0;
        }
        return Magnitude;
    }

    public void mecanumMove(double joystickX, double joystickY, double rotation, boolean slowToggle) {
        double SPEED_REDUCTION;

        if(slowToggle){
            SPEED_REDUCTION = .3;
        } else {
            SPEED_REDUCTION = 1;
        }

        double vMagnitude = getVMagnitude(joystickX, joystickY);
        double vAngle = getVAngle(joystickX, joystickY);

        double LB = vMagnitude * Math.cos(vAngle) + (.7 * rotation);
        double RB = vMagnitude * Math.sin(vAngle) - (.7 * rotation);
        double LF = vMagnitude * Math.sin(vAngle) + (.7 * rotation);
        double RF = vMagnitude * Math.cos(vAngle) - (.7 * rotation);

        LF *= SPEED_REDUCTION;
        LB *= SPEED_REDUCTION;
        RF *= SPEED_REDUCTION;
        RB *= SPEED_REDUCTION;

        LF *= Math.abs(LF);
        LB *= Math.abs(LB);
        RF *= Math.abs(RF);
        RB *= Math.abs(RB);

        LF = clamp(LF);
        LB = clamp(LB);
        RF = clamp(RF);
        RB = clamp(RB);

        RFmotor.setPower(RF);
        RBmotor.setPower(RB);
        LFmotor.setPower(LF); 
        LBmotor.setPower(LB);

    }

    public double getPitch() {
        Orientation angOrientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angOrientation.firstAngle;
    }

    public Orientation getOrientation() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

//    public double[] getColor() {
//        double[] color = new double[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
//        return color;
//    }

    public void resetEncoders() {
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        mineralLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SweeperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        mineralLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        SweeperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //mineralLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        mineralLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        SweeperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double clamp(double d) {
        if(d > 1) {
            d = 1;
        } else if (d < -1) {
            d = -1;
        }
        return  d;
    }

    public double keepLevel(int motorPosition) {
        final int TICKS_PER_REV = 5264;
        final double SERVO_DEGREE = 180;
        double servoDelta = SERVO_DEGREE/TICKS_PER_REV;
        return (motorPosition * servoDelta);
    }

}
