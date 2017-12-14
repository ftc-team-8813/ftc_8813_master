package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
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
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
