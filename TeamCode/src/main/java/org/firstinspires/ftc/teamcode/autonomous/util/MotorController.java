package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.Closeable;

/**
 * Created by aidan on 12/17/17.
 */

public class MotorController implements Closeable {
    private static class ParallelController implements Runnable {

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
                    double speed = error*kP + integral*kI + derivative*kD;
                    speed = Utils.constrain(speed, -1, 1);
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
    }
    private Thread thread;
    private ParallelController controller;
    private boolean closed = false;

    public MotorController(DcMotor motor) {
        controller = new ParallelController(motor);
        thread = new Thread(controller);
        thread.start();
    }

    public void hold(int position) {
        if (closed) throw new RuntimeException("Motor controller closed");
        controller.setTarget(position);
        controller.hold();
    }

    public int getTargetPosition() {
        if (closed) throw new RuntimeException("Motor controller closed");
        return controller.getTarget();
    }

    public int getCurrentPosition() {
        if (closed) throw new RuntimeException("Motor controller closed");
        return controller.getCurrentPosition();
    }

    public void stopHolding() {
        if (closed) throw new RuntimeException("Motor controller closed");
        controller.stopHolding();
    }

    public void runToPosition(int position) throws InterruptedException {
        if (closed) throw new RuntimeException("Motor controller closed");
        runToPosition(position, true);
    }

    public void runToPosition(int position, boolean keepHolding) throws InterruptedException {
        if (closed) throw new RuntimeException("Motor controller closed");
        hold(position);
        while (!controller.nearTarget(5)) {
            Thread.sleep(10);
        }
        if (!keepHolding) stopHolding();
    }

    public double[] getPIDConstants() {
        if (closed) throw new RuntimeException("Motor controller closed");
        return controller.getPIDConstants();
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        if (closed) throw new RuntimeException("Motor controller closed");
        controller.setPIDConstants(kP, kI, kD);
    }

    public void close() {
        if (closed) throw new RuntimeException("Motor controller closed");
        thread.interrupt();
    }

    public boolean closed() {
        return closed;
    }
}
