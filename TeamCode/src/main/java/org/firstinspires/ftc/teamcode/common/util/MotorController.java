package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.util.concurrent.ResettableCountDownLatch;

import java.io.Closeable;
import java.io.File;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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
        protected volatile double minPower = -1, maxPower = 1;
        protected Logger log;
        protected volatile boolean holding = false;
        protected volatile boolean stopNearTarget = false;
        protected volatile boolean holdStalled = true;
        protected DcMotor motor;
        protected Runnable atTarget;
        private boolean stopping;
        protected PIDController controller;
        
        private long stall_begin;
        private DataLogger datalogger;
        
        public final int deadband;

        private double last_speed;

        private ResettableCountDownLatch latch = new ResettableCountDownLatch(1);
        
        ParallelController(DcMotor motor, int deadband, boolean noReset)
        {
            this.deadband = deadband;
            log = new Logger("Motor " + motor.getPortNumber() + " Controller");
            log.d("Initializing!");
            this.motor = motor;
            if (!noReset)
            {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            controller = new PIDController(0, 0, 0, false);
            datalogger = new DataLogger(
                    new File(Config.storageDir + "pidLog_motor" + motor.getPortNumber() + ".dat"),
                    new DataLogger.Channel("target",   0xFFFF00),
                    new DataLogger.Channel("position", 0x00FF00),
                    new DataLogger.Channel("error",    0xFF0000),
                    new DataLogger.Channel("integral", 0x7F00FF),
                    new DataLogger.Channel("deriv",    0x00FFFF),
                    new DataLogger.Channel("output",   0xFFFFFF));
        }
        
        @Override
        public void run()
        {
            log.i("Starting!");
            try
            {
                // Stagger the thread run-times to avoid blocking the processor at regular intervals
                Thread.sleep(2 * motor.getPortNumber());
            }
            catch (InterruptedException e)
            {
                return;
            }
            while (true)
            {
                if (holding)
                {
                    if (stopping) latch.countDown();
                    stopping = false;
                    if (!holdStalled && controller.getDerivative() == 0
                            && Math.abs(controller.getError()) > deadband
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
                    speed = Utils.constrain(speed, minPower, maxPower);
                    if (Math.abs(controller.getError()) < deadband && Math.abs(controller.getDerivative()) < 2)
                    {
                        if (stopNearTarget)
                        {
                            stopHolding();
                            stopNearTarget = false;
                        }
                        speed = 0;
                        controller.resetIntegrator();
                        if (atTarget != null) atTarget.run();
                    }
                    if (speed != last_speed)
                    {
                        last_speed = speed;
                        motor.setPower(speed);
                    }
                    controller.integrate(speed); // I think this is what they mean by integrating the feed-forward
                } else
                {
                    if (!stopping)
                    {
                        log.d("Stopping motor controller");
                        latch.countDown();
                        stopping = true;
                        motor.setPower(0);
                        controller.resetIntegrator();
                    }
                }

                /*
                double[] data = {
                        getTarget(),
                        getCurrentPosition(),
                        controller.getError(),
                        controller.getIntegral(),
                        controller.getDerivative(),
                        controller.getOutput()
                };
                datalogger.log(data);
                */

                try
                {
                    Thread.sleep(10);
                }
                catch (InterruptedException e)
                {
                    motor.setPower(0);
                    controller.resetIntegrator();
                    datalogger.close();
                    return;
                }
            }
        }

        void startLogging()
        {
            datalogger.startClip();
        }
        
        void setTarget(int target)
        {
            if (controller.getTarget() != target)
            {
                log.d("Position set to %d", target);
                controller.setTarget(target);
            }
        }

        void runAtTarget(Runnable atTarget)
        {
            this.atTarget = atTarget;
        }
        
        int getTarget()
        {
            return (int)controller.getTarget();
        }
        
        void hold()
        {
            if (!holding)
            {
                log.d("Holding position @ power = %.4f", power);
                latch.reset();
                holding = true;
                try { latch.await(); } catch (InterruptedException e) {}
            }
        }
        
        void stopHolding()
        {
            if (holding)
            {
                log.d("Stop holding position");
                latch.reset();
                holding = false;
                try { latch.await(); } catch (InterruptedException e) {}
                log.d("Stopped holding position");
            }
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
            if (controller.getPIDConstants()[0] != kP ||
                controller.getPIDConstants()[1] != kI ||
                controller.getPIDConstants()[2] != kD)
            {
                log.i("Updated PID constants to kP = %.4f, kI = %.4f, kD = %.4f", kP, kI, kD);
            }
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

        void constrainPower(double minPower, double maxPower)
        {
            this.minPower = minPower;
            this.maxPower = maxPower;
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
    
    private MotorController(ParallelController controller, double[] constants)
    {
        this.controller = controller;
        controller.setPIDConstants(constants);
        // Create the motor controller thread
        thread = new Thread(controller, "Motor " + controller.motor.getPortNumber() + " controller thread");
        thread.start();
    }

    public static class MotorControllerFactory
    {
        private DcMotor motor;
        private int deadband = 5;
        private double[] constants = new double[3];
        private boolean noReset = true;

        public MotorControllerFactory(DcMotor motor)
        {
            this.motor = motor;
        }

        public MotorControllerFactory setDeadband(int deadband)
        {
            this.deadband = deadband;
            return this;
        }

        public MotorControllerFactory setConstants(double[] constants)
        {
            this.constants = constants;
            return this;
        }

        public MotorControllerFactory setConstants(double kP, double kI, double kD)
        {
            this.constants = new double[] { kP, kI, kD };
            return this;
        }

        public MotorControllerFactory resetEncoderOnInit(boolean reset)
        {
            this.noReset = !reset;
            return this;
        }

        public MotorController create()
        {
            return new MotorController(new ParallelController(motor, deadband, noReset), constants);
        }
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
     * Resume holding the last position that was set.
     *
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void resumeHolding()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.hold();
    }
    
    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position. Keeps holding after reaching the position.
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
        while (!controller.nearTarget(controller.deadband))
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

    /**
     * Access the internal {@link PIDController}. Use this to access data for logging.
     * @return The internal PIDController
     */
    public PIDController getInternalController()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getInternalController();
    }

    /**
     * Get the current motor output. NOTE that this is different from {@link PIDController#getOutput()}
     * because the motor controller does some adjustments (specifically, power and steady-state error).
     * @return The current motor power
     */
    public double getOutput()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.motor.getPower();
    }

    /**
     * Tell the controller whether to keep holding the motor when it appears to be stalled.
     * @param hold If true, keep holding when stalled. Otherwise, stop after 2 seconds of stall.
     */
    public void holdStalled(boolean hold)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.holdStalled = hold;
    }

    /**
     * Enable motor controller logging. NOTE that a log file will be generated regardless of whether
     * this function is called!
     */
    public void startLogging()
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.startLogging();
    }

    /**
     * Constrain the absolute power that can be delivered to the motor.
     * @param min Minimum power (in the range -1..1)
     * @param max Maximum power (in the range -1..1)
     */
    public void constrainPower(double min, double max)
    {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.constrainPower(min, max);
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
