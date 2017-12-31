package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * DcMotorUtil - Utility class for moving DC motors to specific positions.
 */

public class DcMotorUtil {

    private static Map<DcMotor, MotorController> controllers = new HashMap<>();

    public static int degreesToEncoders(double degrees, int gearRatio) {
        return (int)Utils.scaleRange(degrees, 0, 360, 0, 28 * gearRatio);
    }

    public static double encodersToDegrees(int encoders, int gearRatio) {
        return Utils.scaleRange(encoders, 0, 28 * gearRatio, 0, 360);
    }

    public static void holdPosition(DcMotor motor, int encoderValue, double power) {
        MotorController controller;
        if ((controller = controllers.get(motor)) == null) {
            controller = new MotorController(motor);
            controllers.put(motor, controller);
        }
        controller.hold(encoderValue);
    }

    public static void moveToPosition(DcMotor motor, int encoderValue, double power) throws InterruptedException {
        holdPosition(motor, encoderValue, power);
        Telemetry.Item[] items = new Telemetry.Item[2];
        while (motor.isBusy()) {
            items[0] = BaseAutonomous.instance().telemetry.addData("Encoder count", motor.getCurrentPosition());
            items[1] = BaseAutonomous.instance().telemetry.addData("Target position", encoderValue);
            BaseAutonomous.instance().telemetry.update();
            Thread.sleep(10);
        }
        BaseAutonomous.instance().telemetry.removeItem(items[0]);
        BaseAutonomous.instance().telemetry.removeItem(items[1]);
        stopHoldingPosition(motor);
    }

    public static void stopHoldingPosition(DcMotor motor) {
        MotorController controller;
        if ((controller = controllers.get(motor)) == null) return;
        controllers.remove(controller).close();
    }

    private static class Controller {
        private DcMotor motor;
        private volatile int error;
        private volatile boolean running;
        public Controller(DcMotor motor) {
            this.motor = motor;
        }

        public void startHolding(int position) {
            running = true;
            Thread controller = new Thread(new Runnable() {
                @Override
                public void run() {
                    while (running) {

                    }
                }
            }, motor.getDeviceName() + " motor controller thread");
        }

        public void stopHolding() {
            running = false;
        }

        public DcMotor getControllingMotor() {
            return motor;
        }

        public boolean isRunning() {
            return running;
        }

        public int getError() {
            return error;
        }
    }
}
