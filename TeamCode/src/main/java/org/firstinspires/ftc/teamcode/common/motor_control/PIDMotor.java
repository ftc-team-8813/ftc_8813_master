package org.firstinspires.ftc.teamcode.common.motor_control;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataLogger;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.PIDController;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.firstinspires.ftc.teamcode.common.util.concurrent.ResettableCountDownLatch;

import java.io.Closeable;
import java.io.File;

/**
 * Wrapper for the built-in DC motor controller; breaks out more functionality than DcMotor.
 */

public class PIDMotor implements Closeable
{
    
    private DcMotor motor;
    private LynxDcMotorController controller;
    private int port;
    
    private int deadband = 20; // For checking if the motor is busy

    private double power = 0.2;
    
    private Logger log;
    
    public PIDMotor(DcMotor motor)
    {
        this.motor = motor;
        DcMotorController controller = motor.getController();
        if (!(controller instanceof LynxDcMotorController))
        {
            throw new IllegalArgumentException("PIDMotor only works with the REV motor controller!");
        }
        this.controller = (LynxDcMotorController)controller;
        this.port = motor.getPortNumber();
        log = new Logger("PIDMotor " + Utils.getMotorId(motor));
    
        GlobalDataLogger.instance().addChannel(Utils.getMotorId(motor) + " position",
                () -> Integer.toString(motor.getCurrentPosition()));
        GlobalDataLogger.instance().addChannel(Utils.getMotorId(motor) + " target",
                () -> isHolding() ? Integer.toString(motor.getTargetPosition()) : "Idle");
    }
    
    public void setDeadband(int deadband)
    {
        this.deadband = deadband;
    }
    
    /**
     * Get the DcMotor that this PIDMotor is/was controlling
     * @return The underlying DcMotor, even if the controller is closed
     */
    public DcMotor getMotor()
    {
        return motor;
    }

    public void setRunMode(DcMotor.RunMode value){
        controller.setMotorMode(port, value);
    }
    
    /**
     * Start holding a position. Returns immediately. Since the default power is zero, this function
     * won't do anything until {@link #setPower(double) } is called.
     *
     * @param position The position to hold
     * @see #stopHolding()
     * @see #runToPosition(int)
     * @see #runToPosition(int, boolean)
     */
    public void hold(int position)
    {
        // We have to manually adjust the position since we're using the controller function
        if (motor.getDirection() == DcMotorSimple.Direction.REVERSE) position = -position;
        
        log.v("hold(%d) at power %.3f (current position %d)", position, power, getCurrentPosition());
        controller.setMotorTargetPosition(port, position, deadband);
        controller.setMotorMode(port, DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    
    
    /**
     * DOES NOT SET THE MOTOR SPEED!
     * Set the maximum absolute power which can be attained when trying to hold a position.
     * Takes the absolute value of the input, so a power of -1 would be the same as a power of 1.
     *
     * @param power The maximum power
     */
    public void setPower(double power)
    {
        log.v("setPower(%.3f)", power);
        this.power = power;
        if (controller.getMotorMode(port) == DcMotor.RunMode.RUN_TO_POSITION)
        {
            motor.setPower(power);
        }
    }
    
    /**
     * Return the current position which the controller is trying to hold.
     *
     * @return The target position
     */
    public int getTargetPosition()
    {
        return motor.getTargetPosition();
    }
    
    /**
     * Return the current position of the motor.
     *
     * @return The current position
     */
    public int getCurrentPosition()
    {
        return motor.getCurrentPosition();
    }
    
    /**
     * Stop trying to keep at a certain position.
     */
    public void stopHolding()
    {
        log.v("stopHolding()");
        motor.setPower(0);
        controller.setMotorMode(port, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Resume holding the last position that was set.
     */
    public void resumeHolding()
    {
        log.v("resumeHolding() at power=%.3f", power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    
    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position. Keeps holding after reaching the position.
     *
     * @param position The new position to drive to
     * @throws InterruptedException  if the thread is interrupted while waiting for the motor to
     *                               reach its target
     * @see #hold(int)
     * @see #runToPosition(int, boolean)
     */
    public void runToPosition(int position) throws InterruptedException
    {
        runToPosition(position, true);
    }
    
    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position. If keepHolding is false, the controller will stop holding its
     * position when it reaches its target.
     *
     * @param position    The new position to drive to
     * @param keepHolding Whether to stop the motor when it reaches its target
     * @throws InterruptedException  if the thread is interrupted while waiting for the motor to
     *                               reach its target
     * @see #runToPosition(int)
     */
    public void runToPosition(int position, boolean keepHolding) throws InterruptedException
    {
        hold(position);
        while (motor.getPower() == 0 || controller.isBusy(port))
        {
            Thread.sleep(10);
        }
        if (!keepHolding) stopHolding();
    }
    
    /**
     * Start driving to a position. Does not hold the position once the target has been reached.
     *
     * @param position The position
     * @throws NullPointerException if the GlobalThreadPool hasn't been initialized
     */
    public void startRunToPosition(int position)
    {
        hold(position);
        GlobalThreadPool.instance().start(() ->
        {
           do
           {
               try
               {
                   Thread.sleep(10);
               }
               catch (InterruptedException e)
               {
                   break;
               }
           } while (motor.getPower() == 0 || controller.isBusy(port));
           stopHolding();
        });
    }
    
    /**
     * Get the current PIDF constants as an array of <code>[kP, kI, kD, kF]</code>
     *
     * @param mode Use RUN_TO_POSITION to get the position PID constants and RUN_USING_ENCODER for
     *             the velocity PID constants
     *
     * @return The current PIDF constants for the specified mode
     */
    public double[] getPIDConstants(DcMotor.RunMode mode)
    {
        PIDFCoefficients coeffs = controller.getPIDFCoefficients(port, mode);
        return new double[] {coeffs.p, coeffs.i, coeffs.d, coeffs.f};
    }
    
    /**
     * Backwards compatibility. Returns the position PIDF constants
     * @return The PIDF constants
     */
    public double[] getPIDConstants()
    {
        return getPIDConstants(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    /**
     * Set the PID constants. For backward compatibility, this uses position mode and sets the F constant to zero.
     * Use {@link #setPIDFConstants} if you do not want this.
     *
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     */
    public void setPIDConstants(double kP, double kI, double kD)
    {
        controller.setPIDCoefficients(port, DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(kP, kI, kD));
    }
    
    /**
     * Set the PIDF constants for the given mode (RUN_TO_POSITION or RUN_USING_ENCODER).
     *
     * @param mode The mode
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     * @param kF The feed-forward gain
     */
    public void setPIDFConstants(DcMotor.RunMode mode, double kP, double kI, double kD, double kF)
    {
        controller.setPIDFCoefficients(port, mode, new PIDFCoefficients(kP, kI, kD, kF));
    }
    
    /**
     * Set the direction of the motor. Does not invert the position; instead, adjustments will be in
     * the opposite direction. It is recommended that the motor position be 0 when changing
     * direction.
     *
     * @param reverse If true, this function reverses the motor direction from the default.
     *                Otherwise, it will reset the direction to the default.
     */
    public void setReverse(boolean reverse)
    {
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
    
    /**
     * Alias for {@link #setReverse} that takes a Direction argument instead of a boolean.
     *
     * @param direction If REVERSE, this function reverses the motor direction from the default.
     *                Otherwise, it will reset the direction to the default.
     * @throws IllegalArgumentException if the motor controller has been closed
     */
    public void setDirection(DcMotorSimple.Direction direction)
    {
        setReverse(direction == DcMotorSimple.Direction.REVERSE);
    }
    
    /**
     * Returns whether the motor is holding a position.
     *
     * @return true if the motor is trying to hold a position.
     */
    public boolean isHolding()
    {
        return controller.getMotorMode(port) == DcMotor.RunMode.RUN_TO_POSITION;
    }
    
    /**
     * Returns false when the motor reaches its target.
     * @return Whether the motor is busy
     */
    public boolean isBusy()
    {
        return controller.isBusy(port);
    }

    /**
     * Access the internal {@link PIDController}. This is irrelevant now.
     * @return The internal PIDController
     */
    @Deprecated
    public PIDController getInternalController()
    {
        return null;
    }

    /**
     * Get the current velocity of the motor
     * @return The current motor velocity, in ticks per second
     */
    public double getOutput()
    {
        return controller.getMotorVelocity(port);
    }
    
    /**
     * Get the power of the motor
     * @return How fast the motor is moving (or its maximum speed when holding a position) on a range from -1 to 1
     */
    public double getPower()
    {
        return motor.getPower();
    }
    
    
    public void setVelocity(double velocity)
    {
        controller.setMotorMode(port, DcMotor.RunMode.RUN_USING_ENCODER);
        controller.setMotorVelocity(port, velocity);
    }

    /**
     * TODO add a thread that checks if the motor is stalled
     */
    public void holdStalled(boolean hold)
    {
    
    }

    /**
     * DOES NOTHING
     */
    @Deprecated
    public void startLogging()
    {
    
    }

    /**
     * This is not supported by the REV controller
     * @param min Minimum power (in the range -1..1)
     * @param max Maximum power (in the range -1..1)
     */
    @Deprecated
    public void constrainPower(double min, double max)
    {
    }

    /**
     * Stop the motor. Doesn't do anything else.
     */
    @Deprecated
    public void close()
    {
        motor.setPower(0);
    }
    
    /**
     * Returns false
     */
    @Deprecated
    public boolean closed()
    {
        return false;
    }
}
