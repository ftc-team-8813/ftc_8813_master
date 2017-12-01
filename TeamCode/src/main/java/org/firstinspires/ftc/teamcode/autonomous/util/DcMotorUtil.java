package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * DcMotorUtil - Utility class for moving DC motors to specific positions.
 */

public class DcMotorUtil {

    public static int degreesToEncoders(double degrees, int gearRatio) {
        return (int)Utils.scaleRange(degrees, 0, 360, 0, 28 * gearRatio);
    }

    public static double encodersToDegrees(int encoders, int gearRatio) {
        return Utils.scaleRange(encoders, 0, 28 * gearRatio, 0, 360);
    }

    public static void holdPosition(DcMotor motor, int encoderValue, double power) {
        motor.setTargetPosition(encoderValue);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    public static void moveToPosition(DcMotor motor, int encoderValue, double power) throws InterruptedException {
        holdPosition(motor, encoderValue, power);
        while (motor.isBusy()) {
            Thread.sleep(10);
        }
    }

    public static void stopHoldingPosition(DcMotor motor) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
