package org.firstinspires.ftc.teamcode.robot;

public class PIDController {

    private double P=0;
    private double I=0;
    private double D=0;
    private double F=0;

    private double maxIOutput=0;
    private double maxError=0;
    private double errorSum=0;

    private double maxOutput=0;
    private double minOutput=0;

    private double setpoint=0;

    private double lastActual=0;

    private boolean firstRun=true;
    private boolean reversed=false;

    private double outputRampRate=0;
    private double lastOutput=0;

    private double outputFilter=0;

    private double setpointRange=0;


    public PIDController(double p, double i, double d) {
        P=p; I=i; D=d;
        checkSigns();
    }

    public PIDController(double p, double i, double d, double f) {
        P=p; I=i; D=d; F=f;
        checkSigns();
    }


    //**********************************
    // Configuration functions
    //**********************************

    public void setP(double p) {
        P=p;
        checkSigns();
    }

    public void setI(double i) {
        if (I!=0) {
            errorSum=errorSum*I/i;
        }
        if (maxIOutput!=0) {
            maxError=maxIOutput/i;
        }
        I=i;
        checkSigns();
        // Implementation note:
        // This Scales the accumulated error to avoid output errors.
        // As an example doubling the I term cuts the accumulated error in half, which results in the
        // output change due to the I term constant during the transition.
    }

    public void setD(double d) {
        D=d;
        checkSigns();
    }

    public void setF(double f) {
        F=f;
        checkSigns();
    }

    public void setPID(double p, double i, double d) {
        P=p;D=d;
        //Note: the I term has additional calculations, so we need to use it's
        //specific method for setting it.
        setI(i);
        checkSigns();
    }

    public void setPID(double p, double i, double d,double f) {
        P=p;D=d;F=f;
        //Note: the I term has additional calculations, so we need to use it's
        //specific method for setting it.
        setI(i);
        checkSigns();
    }

    public void setMaxIOutput(double maximum) {
        // Internally maxError and Izone are similar, but scaled for different purposes.
        // The maxError is generated for simplifying math, since calculations against
        // the max error are far more common than changing the I term or Izone.
        maxIOutput=maximum;
        if (I!=0) {
            maxError=maxIOutput/I;
        }
    }

    public void setOutputLimits(double output) {
        setOutputLimits(-output,output);
    }

    public void setOutputLimits(double minimum,double maximum) {
        if (maximum<minimum) return;
        maxOutput=maximum;
        minOutput=minimum;

        // Ensure the bounds of the I term are within the bounds of the allowable output swing
        if (maxIOutput==0 || maxIOutput>(maximum-minimum)) {
            setMaxIOutput(maximum-minimum);
        }
    }

    public void setDirection(boolean reversed) {
        this.reversed=reversed;
    }


    //**********************************
    // Primary operating functions
    //**********************************

    public void setSetpoint(double setpoint) {
        this.setpoint=setpoint;
    }

    public double getOutput(double actual, double setpoint) {
        double output;
        double Poutput;
        double Ioutput;
        double Doutput;
        double Foutput;

        this.setpoint=setpoint;

        // Ramp the setpoint used for calculations if user has opted to do so
        if (setpointRange!=0) {
            setpoint=constrain(setpoint, actual-setpointRange, actual+setpointRange);
        }

        // Do the simple parts of the calculations
        double error=setpoint-actual;

        // Calculate F output. Notice, this depends only on the setpoint, and not the error.
        Foutput=F*setpoint;

        // Calculate P term
        Poutput=P*error;

        // If this is our first time running this, we don't actually _have_ a previous input or output.
        // For sensor, sanely assume it was exactly where it is now.
        // For last output, we can assume it's the current time-independent outputs.
        if (firstRun) {
            lastActual=actual;
            lastOutput=Poutput+Foutput;
            firstRun=false;
        }

        // Calculate D Term
        // Note, this is negative. This actually "slows" the system if it's doing
        // the correct thing, and small values helps prevent output spikes and overshoot
        Doutput= -D*(actual-lastActual);
        lastActual=actual;

        // The Iterm is more complex. There's several things to factor in to make it easier to deal with.
        // 1. maxIoutput restricts the amount of output contributed by the Iterm.
        // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
        // 3. prevent windup by not increasing errorSum if output is output=maxOutput
        Ioutput=I*errorSum;
        if (maxIOutput!=0) {
            Ioutput=constrain(Ioutput, -maxIOutput, maxIOutput);
        }

        // And, finally, we can just add the terms up
        output=Foutput + Poutput + Ioutput + Doutput;

        // Figure out what we're doing with the error.
        if (minOutput!=maxOutput && !bounded(output, minOutput, maxOutput)) {
            errorSum=error;
            // reset the error sum to a sane level
            // Setting to current error ensures a smooth transition when the P term
            // decreases enough for the I term to start acting upon the controller
            // From that point the I term will build up as would be expected
        }
        else if (outputRampRate!=0 && !bounded(output, lastOutput-outputRampRate,lastOutput+outputRampRate)) {
            errorSum=error;
        }
        else if (maxIOutput!=0) {
            errorSum=constrain(errorSum+error, -maxError, maxError);
            // In addition to output limiting directly, we also want to prevent I term
            // buildup, so restrict the error directly
        }
        else{
            errorSum+=error;
        }

        // Restrict output to our specified output and ramp limits
        if (outputRampRate!=0) {
            output=constrain(output, lastOutput-outputRampRate, lastOutput+outputRampRate);
        }
        if (minOutput!=maxOutput) {
            output=constrain(output, minOutput, maxOutput);
        }
        if (outputFilter!=0) {
            output=lastOutput*outputFilter+output*(1-outputFilter);
        }

        // Get a test printline with lots of details about the internal
        // calculations. This can be useful for debugging.
        // System.out.printf("Final output %5.2f [ %5.2f, %5.2f , %5.2f  ], eSum %.2f\n",output,Poutput, Ioutput, Doutput,errorSum );
        // System.out.printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\n",output,Poutput, Ioutput, Doutput );

        lastOutput=output;
        return output;
    }

    public double getOutput() {
        return getOutput(lastActual, setpoint);
    }

    public double getOutput(double actual) {
        return getOutput(actual, setpoint);
    }

    public void reset() {
        firstRun=true;
        errorSum=0;
    }

    public void setOutputRampRate(double rate) {
        outputRampRate=rate;
    }

    public void setSetpointRange(double range) {
        setpointRange=range;
    }

    public void setOutputFilter(double strength) {
        if (strength==0 || bounded(strength,0,1)) {
            outputFilter=strength;
        }
    }


    //**************************************
    // Helper functions
    //**************************************

    private double constrain(double value, double min, double max) {
        if (value > max) { return max;}
        if (value < min) { return min;}
        return value;
    }

    private boolean bounded(double value, double min, double max) {
        // Note, this is an inclusive range. This is so tests like
        // bounded(constrain(0,0,1),0,1)` will return false.
        // This is more helpful for determining edge-case behaviour
        // than <= is.
        return (min<value) && (value<max);
    }

    private void checkSigns() {
        if (reversed) {  // all values should be below zero
            if (P>0) P*=-1;
            if (I>0) I*=-1;
            if (D>0) D*=-1;
            if (F>0) F*=-1;
        }
        else{  // all values should be above zero
            if (P<0) P*=-1;
            if (I<0) I*=-1;
            if (D<0) D*=-1;
            if (F<0) F*=-1;
        }
    }
}