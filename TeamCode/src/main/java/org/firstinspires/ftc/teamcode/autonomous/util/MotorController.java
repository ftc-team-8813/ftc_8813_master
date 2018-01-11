package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.Closeable;

/**
 * DC motor controller. Replacement for the buggy built-in REV motor controller.
 */

public class MotorController implements Closeable {
    /*
    Internal motor controller. Runs in parallel with the main controller.
     */
    private static class ParallelController implements Runnable {

        private volatile double power = 1;
        private volatile int target;
        private volatile boolean holding = false;
        private volatile double kP, kI, kD;
        private DcMotor motor;
        private double integral;
        private double prev_error;

        ParallelController(DcMotor motor) {
            this.motor = motor;
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            target = 0;
            kP = 0;
            kI = 0;
            kD = 0;
        }

        @Override
        public void run() {
            while (true) {
                if (holding) {
                    double error = target - motor.getCurrentPosition(); //target > pos : error positive
                    integral += error;
                    double derivative = error - prev_error;
                    prev_error = error;
                    double speed = error*kP + integral*(kI/1000) + derivative*kD;
                    speed = Utils.constrain(speed, -1, 1) * power;
                    if (Math.abs(error) < 5) {
                        integral -= error;
                        speed = 0;
                    }
                    motor.setPower(speed);
                } else {
                    motor.setPower(0);
                    integral = 0;
                }
                if (Thread.interrupted()) {
                    motor.setPower(0);
                    integral = 0;
                    return;
                }
            }
        }

        void setTarget(int target) {
            this.target = target;
        }

        int getTarget() {
            return target;
        }

        void hold() {
            holding = true;
        }

        void stopHolding() {
            holding = false;
        }

         boolean isHolding() {
            return holding;
        }

        int getCurrentPosition() {
            return motor.getCurrentPosition();
        }

        boolean nearTarget(int error) {
            return Math.abs(target - motor.getCurrentPosition()) > error;
        }

        double[] getPIDConstants() {
            return new double[] {kP, kI, kD};
        }

        void setPIDConstants(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        void setPIDConstants(double[] constants) {
            setPIDConstants(constants[0], constants[1], constants[2]);
        }
    }
    /* Thread that runs the ParallelControler */
    private Thread thread;
    /* The ParallelController */
    private ParallelController controller;
    /* Whether or not the controller has been closed */
    private boolean closed = false;

    /**
     * Create a motor controller to control the specified motor. Uses the specified Config to load
     * PID constants.
     * @param motor The motor to control
     * @param conf The Config to get PID constants from
     * @see #MotorController(DcMotor)
     */
    public MotorController(DcMotor motor, Config conf) {
        controller = new ParallelController(motor);
        controller.setPIDConstants(conf.getDoubleArray("pid_constants"));
        thread = new Thread(controller);
        thread.start();

    }

    /**
     * Create a motor controller to control the specified motor. Uses BaseAutonomous.config to get
     * PID constants, so when in TeleOp, this constructor will fail. Please specify a Config to use
     * in TeleOp.
     * @param motor The motor to control
     * @see #MotorController(DcMotor, Config)
     */
    public MotorController(DcMotor motor) {
        this(motor, BaseAutonomous.instance().config);
    }

    /**
     * Start holding a position. Returns immediately.
     * @param position The position to hold
     * @throws IllegalStateException if the motor controller has been closed
     * @see #stopHolding()
     * @see #runToPosition(int)
     * @see #runToPosition(int, boolean)
     */
    public void hold(int position) {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.setTarget(position);
        controller.hold();
    }

    /**
     * Set the maximum absolute power which can be attained when trying to hold a position.
     * Constrained between 0 and 1.
     * @param power The maximum power
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void setPower(double power) {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.power = Utils.constrain(power, 0, 1);
    }

    /**
     * Return the current position which the controller is trying to hold.
     * @return The target position
     * @throws IllegalStateException if the motor controller has been closed
     */
    public int getTargetPosition() {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getTarget();
    }

    /**
     * Return the current position of the motor.
     * @return The current position
     * @throws IllegalStateException if the motor controller has been closed
     */
    public int getCurrentPosition() {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getCurrentPosition();
    }

    /**
     * Stop trying to keep at a certain position.
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void stopHolding() {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.stopHolding();
    }

    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position.
     * @param position The new position to drive to
     * @throws InterruptedException if the thread is interrupted while waiting for the motor to
     *                              reach its target
     * @throws IllegalStateException if the motor controller has been closed
     * @see #hold(int)
     * @see #runToPosition(int, boolean)
     */
    public void runToPosition(int position) throws InterruptedException {
        if (closed) throw new IllegalStateException("Motor controller closed");
        runToPosition(position, true);
    }

    /**
     * Start holding a certain position. Blocks until the motor position is within 5 encoder counts
     * of the target position. If keepHolding is false, the controller will stop holding its
     * position when it reaches its target.
     * @param position The new position to drive to
     * @param keepHolding Whether to stop the motor when it reaches its target
     * @throws InterruptedException if the thread is interrupted while waiting for the motor to
     *                              reach its target
     * @throws IllegalStateException if the motor controller has been closed
     *
     * @see #runToPosition(int)
     */
    public void runToPosition(int position, boolean keepHolding) throws InterruptedException {
        if (closed) throw new IllegalStateException("Motor controller closed");
        hold(position);
        while (!controller.nearTarget(5)) {
            Thread.sleep(10);
        }
        if (!keepHolding) stopHolding();
    }

    /**
     * Get the current PID constants as an array of <code>[kP, kI, kD]</code>
     * @return The current PID constants
     * @throws IllegalStateException if the motor controller has been closed
     */
    public double[] getPIDConstants() {
        if (closed) throw new IllegalStateException("Motor controller closed");
        return controller.getPIDConstants();
    }

    /**
     * Set the PID constants
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     * @throws IllegalStateException if the motor controller has been closed
     */
    public void setPIDConstants(double kP, double kI, double kD) {
        if (closed) throw new IllegalStateException("Motor controller closed");
        controller.setPIDConstants(kP, kI, kD);
    }

    /**
     * Close the motor controller. Attempting to access any of the methods (except
     * {@link #closed()}) after the controller has been closed will throw an IllegalStateException.
     * @throws IllegalStateException if the motor controller has already been closed
     */
    public void close() {
        if (closed) throw new IllegalStateException("Motor controller closed");
        thread.interrupt();
    }

    /**
     * Returns whether the controller has been closed
     * @return true if the controller is closed
     */
    public boolean closed() {
        return closed;
    }
}
