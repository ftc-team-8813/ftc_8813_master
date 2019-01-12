package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.PIDController;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.Closeable;

/**
 * DC motor controller. Replacement for the buggy built-in REV motor controller.
 */

public class MotorController implements Closeable
{
    /*
    Internal motor controller. Runs in parallel with the main controller.
     */
    protected static class ParallelController implements Runnable
    {
        
        protected volatile double power = 1;
        protected Logger log;
        protected volatile boolean holding = false;
        protected volatile boolean stopNearTarget = false;
        protected volatile boolean holdStalled = true;
        protected DcMotor motor;
        protected Runnable atTarget;
        private boolean stopping;
        protected PIDController controller;
        
        private long stall_begin;
        
        public final int sse;
        
        ParallelController(DcMotor motor, Runnable atTarget, int steady_state_error, boolean
                noReset)
        {
            this.sse = steady_state_error;
            log = new Logger("Motor " + motor.getPortNumber() + " Controller");
            log.d("Initializing!");
            this.motor = motor;
            if (!noReset)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            controller = new PIDController(0, 0, 0);
        }
        
        @Override
        public void run()
        {
            log.i("Starting!");
            while (true)
            {
                if (holding)
                {
                    stopping = false;
                    if (!holdStalled && controller.getDerivative() == 0
                            && Math.abs(controller.getError()) > sse
                            && Math.abs(motor.getPower()) > 0.6)
                    {
                        if (stall_begin == 0)
                        {
                            stall_begin = System.currentTimeMillis();
                        } else if (System.currentTimeMillis() > stall_begin + 2000)
                        {
                            stopHolding();
                            stopNearTarget = false;
                        }
                    } else
                    {
                        stall_begin = 0;
                    }

                    double speed = controller.process(getCurrentPosition());
                    speed = Utils.constrain(speed, -1, 1) * power;
                    if (Math.abs(controller.getError()) < sse)
                    {
                        if (stopNearTarget && Math.abs(controller.getDerivative()) < 2)
                        {
                            stopHolding();
                            stopNearTarget = false;
                        }
                        speed = 0;
                        if (atTarget != null) atTarget.run();
                    }
                    motor.setPower(speed);
                } else
                {
                    if (!stopping)
                    {
                        stopping = true;
                        motor.setPower(0);
                        controller.resetIntegrator();
                    }
                }
                if (Thread.interrupted())
                {
                    motor.setPower(0);
                    controller.resetIntegrator();
                    return;
                }
            }
        }
        
        void setTarget(int target)
        {
            controller.setTarget(target);
            log.d("Position set to %d", target);
        }
        
        int getTarget()
        {
            return (int)controller.getTarget();
        }
        
        void hold()
        {
            holding = true;
            log.d("Holding position @ power = %.4f", power);
        }
        
        void stopHolding()
        {
            holding = false;
            log.d("Stop holding position");
        }
        
        boolean isHolding()
        {
            return holding;
        }
        
        void stopWhenComplete(boolean stopNearTarget)
        {
            this.stopNearTarget = stopNearTarget;
        }
        
        int getCurrentPosition()
        {
            return motor.getCurrentPosition();
        }
        
        boolean nearTarget(int error)
        {
            return Math.abs(controller.getError()) <= error;
        }
        
        double[] getPIDConstants()
        {
            return controller.getPIDConstants();
        }
        
        void setPIDConstants(double kP, double kI, double kD)
        {
            log.i("Updated PID constants to kP = %.4f, kI = %.4f, kD = %.4f", kP, kI, kD);
            controller.setPIDConstants(kP, kI, kD);
        }
        
        void setPIDConstants(double[] constants)
        {
            setPIDConstants(constants[0], constants[1], constants[2]);
        }
        
        void setReverse(boolean reverse)
        {
            motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        PIDController getInternalController()
        {
            return controller;
        }
    }
    
    /* Thread that runs the ParallelControler */
    private Thread thread;
    /* The ParallelController */
    protected ParallelController controller;
    /* Whether or not the controller has been closed */
    private boolean closed = false;
    
    protected MotorController(ParallelController controller, Config conf, String constants)
    {
        this.controller = controller;
        if (conf != null) this.controller.setPIDConstants(conf.getDoubleArray(constants));
        thread = new Thread(this.controller, "Motor " + controller.motor + " controller " +
                "thread");
        thread.start();
    }
    
    /**
     * Create a motor controller to control the specified motor. Runs atTarget when the motor
     * reaches its target position (e.g. to stop the motor controller). Uses the specified Config to load
     * PID constants.
     *
     * @param motor    The motor to control
     * @param conf     The Config to get PID constants from
     * @param atTarget The Runnable to run repeatedly when the motor is at its target position
     * @param noReset  If true, does not reset the motor encoder.
     */
    public MotorController(DcMotor motor, Config conf, Runnable atTarget, boolean noReset)
    {
        this(new ParallelController(motor, atTarget, conf.getInt("steady_state_error", 0), noReset),
                conf, "pid_constants");
    }
    
    /**
     * Create a motor controller to control the specified motor. Runs atTarget when the motor
     * reaches its target position (e.g. to stop the motor controller). Uses the specified Config to load
     * PID constants.
     *
     * @param motor    The motor to control
     * @param conf     The Config to get PID constants from
     * @param atTarget The Runnable to run repeatedly when the motor is at its target position
     */
    public MotorController(DcMotor motor, Config conf, Runnable atTarget)
    {
        this(motor, conf, atTarget, false);
    }
    
    /**
     * Create a motor controller to control the specified motor. Runs atTarget when the motor
     * reaches its target position (e.g. to stop the motor controller).
     *
     * @param motor    The motor to control
     * @param atTarget The Runnable to run repeatedly when the motor is at its target position
     */
    public MotorController(DcMotor motor, Runnable atTarget)
    {
        this(motor, null, atTarget);
    }
    
    /**
     * Create a motor controller to control the specified motor. Uses the specified Config to load
     * PID constants.
     *
     * @param motor The motor to control
     * @param conf  The Config to get PID constants from
     * @see #MotorController(DcMotor)
     */
    public MotorController(DcMotor motor, Config conf)
    {
        this(motor, conf, null);
    }
    
    /**
     * Create a motor controller to control the specified motor. Uses BaseAutonomous.config to get
     * PID constants, so when in TeleOp, this constructor will fail. Please specify a Config to use
     * in TeleOp.
     *
     * @param motor The motor to control
     * @see #MotorController(DcMotor, Config)
     */
    public MotorController(DcMotor motor)
    {
        this(motor, BaseAutonomous.instance().config);
    }
    
    /**
     * Start holding a position. Returns immediately.
     *
     * @param position The position to hold
     * @throws IllegalStateException if the motor controller has been closed
     * @see #stopHolding()
     * @see #runToPosition(int)
     * @see #runToPosition(int, boolean)
     */
    public void hold(int position)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.stopWhenComplete(false);
        controller.setTarget(position);
        controller.hold();
    }
    
    /**
     * Set the maximum absolute power which can be attained when trying to hold a position.
     * Constrained between 0 and 1.
     *
     * @param power The maximum power
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void setPower(double power)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.power = Utils.constrain(power, 0, 1);
    }
    
    /**
     * Return the current position which the controller is trying to hold.
     *
     * @return The target position
     * @throws IllegalStateException if the motor controller has been closed
     */
    public int getTargetPosition()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getTarget();
    }
    
    /**
     * Return the current position of the motor.
     *
     * @return The current position
     * @throws IllegalStateException if the motor controller has been closed
     */
    public int getCurrentPosition()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getCurrentPosition();
    }
    
    /**
     * Stop trying to keep at a certain position.
     *
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void stopHolding()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.stopHolding();
    }
    
    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position.
     *
     * @param position The new position to drive to
     * @throws InterruptedException  if the thread is interrupted while waiting for the motor to
     *                               reach its target
     * @throws IllegalStateException if the motor controller has been closed
     * @see #hold(int)
     * @see #runToPosition(int, boolean)
     */
    public void runToPosition(int position) throws InterruptedException
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
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
     * @throws IllegalStateException if the motor controller has been closed
     * @see #runToPosition(int)
     */
    public void runToPosition(int position, boolean keepHolding) throws InterruptedException
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        hold(position);
        while (!controller.nearTarget(controller.sse))
        {
            Thread.sleep(10);
        }
        if (!keepHolding) stopHolding();
    }
    
    /**
     * Start driving to a position. Does not hold the position once the target has been reached.
     *
     * @param position The position
     */
    public void startRunToPosition(int position)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.stopWhenComplete(true);
        controller.setTarget(position);
        controller.hold();
    }
    
    /**
     * Get the current PID constants as an array of <code>[kP, kI, kD]</code>
     *
     * @return The current PID constants
     * @throws IllegalStateException if the motor controller has been closed
     */
    public double[] getPIDConstants()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getPIDConstants();
    }
    
    /**
     * Set the PID constants
     *
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void setPIDConstants(double kP, double kI, double kD)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.setPIDConstants(kP, kI, kD);
    }
    
    /**
     * Set the direction of the motor. Does not invert the position; instead, adjustments will be in
     * the opposite direction. It is recommended that the motor position be 0 when changing
     * direction.
     *
     * @param reverse If true, this function reverses the motor direction from the default.
     *                Otherwise, it will reset the direction to the default.
     * @throws IllegalArgumentException if the motor controller has been closed
     */
    public void setReverse(boolean reverse)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.setReverse(reverse);
    }
    
    /**
     * Returns whether the motor is holding a position.
     *
     * @return true if the motor is trying to hold a position.
     */
    public boolean isHolding()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.isHolding();
    }


    public PIDController getInternalController()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getInternalController();
    }

    public double getOutput()
    {
        return controller.motor.getPower();
    }

    public void holdStalled(boolean hold)
    {
        controller.holdStalled = hold;
    }
    /**
     * Close the motor controller. Attempting to access any of the methods (except
     * {@link #closed()}) after the controller has been closed will throw an IllegalStateException.
     *
     * @throws IllegalStateException if the motor controller has already been closed
     */
    public void close()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        thread.interrupt();
    }
    
    /**
     * Returns whether the controller has been closed
     *
     * @return true if the controller is closed
     */
    public boolean closed()
    {
        return closed;
    }
}
