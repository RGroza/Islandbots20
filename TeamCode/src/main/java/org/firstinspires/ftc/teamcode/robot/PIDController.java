package org.firstinspires.ftc.teamcode.robot;

public class PIDController {
    private double kP, kI, kD;
    private double targetAngle, currentAngle;
    private double totalError, lastError;

    public PIDController(double kP, double kI, double kD, double targetAngle) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.targetAngle = targetAngle;
    }

    private void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    private void setTotalError(double value) {
        this.totalError = value;
    }

    private void setLastError(double value) {
        this.lastError = value;
    }

    private double[] getCorrection(double currentAngle) {
        double currentError = currentAngle - targetAngle;



        double[] returnedValues = {0, 0};
        return returnedValues;
    }
}
