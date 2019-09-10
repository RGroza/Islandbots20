package org.firstinspires.ftc.islandbots19.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBotInit {
    public BNO055IMU gyro;
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor;
    public ColorSensor RcolorSensor;
    public ColorSensor LcolorSensor;

    public TestBotInit(HardwareMap hwMap, Telemetry telemetry) {
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

        // sensors
        RcolorSensor = hwMap.colorSensor.get("RcolorSensor");
        LcolorSensor = hwMap.colorSensor.get("LcolorSensor");
        gyro = hwMap.get(BNO055IMU.class, "gyro");

        // motor encoders init
        resetEncoders();

        // motor directions
        LBmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);
        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        RFmotor.setDirection(DcMotor.Direction.REVERSE);

        gyro.initialize(parameters);
        telemetry.addData("Successfully Initialized", null);
        telemetry.update();
    }

    public double[] getColor(boolean isRight) {
        double[] color = new double[]{
                isRight ? RcolorSensor.red() : LcolorSensor.red(),
                isRight ? RcolorSensor.green() : LcolorSensor.green(),
                isRight ? RcolorSensor.blue() : LcolorSensor.blue()
        };
        return color;
    }

    public void resetEncoders() {
        LFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
