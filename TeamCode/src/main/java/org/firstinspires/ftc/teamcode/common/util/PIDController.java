package org.firstinspires.ftc.teamcode.common.util;

public class PIDController
{
    private volatile double target;
    private volatile double error;
    private volatile double derivative;
    private volatile double lastOutput;
    private volatile double kP, kI, kD;
    private final double integratorCutoff;

    private volatile double integral, prevError;

    public PIDController(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        integratorCutoff = .8;
    }

    public synchronized double process(double input)
    {
        error = target - input;

        derivative = error - prevError;
        prevError = error;
        integral += error;

        if (integral * kI > integratorCutoff)
        {
            integral = integratorCutoff;
        }
        else if (integral * kI < -integratorCutoff)
        {
            integral = -integratorCutoff;
        }

        lastOutput = error * kP + integral * kI + derivative * kD;
        return lastOutput;
    }

    public double getOutput()
    {
        return lastOutput;
    }

    public synchronized void setPIDConstants(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double[] getPIDConstants()
    {
        return new double[] {kP, kI, kD};
    }

    public synchronized void setTarget(double target)
    {
        this.target = target;
    }

    public double getTarget()
    {
        return target;
    }

    public double getError()
    {
        return error;
    }

    public double getIntegral()
    {
        return integral;
    }

    public double getDerivative()
    {
        return derivative;
    }

    public synchronized void resetIntegrator()
    {
        integral = 0;
    }
}
