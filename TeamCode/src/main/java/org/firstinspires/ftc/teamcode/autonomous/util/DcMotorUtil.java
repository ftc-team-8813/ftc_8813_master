package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.util.HashMap;
import java.util.Map;

/**
 * DcMotorUtil - Utility class for moving DC motors to specific positions.
 *
 * @Deprecated Resets encoder every move
 */
@Deprecated
public class DcMotorUtil
{
    
    private static Map<DcMotor, MotorController> controllers = new HashMap<>();
    
    /**
     * Convert degrees to encoder ticks.
     *
     * @param degrees   The number of degrees
     * @param gearRatio The gear ratio
     * @return The encoder count re
     */
    public static int degreesToEncoders(double degrees, int gearRatio)
    {
        return (int) Utils.scaleRange(degrees, 0, 360, 0, 28 * gearRatio);
    }
    
    public static double encodersToDegrees(int encoders, int gearRatio)
    {
        return Utils.scaleRange(encoders, 0, 28 * gearRatio, 0, 360);
    }
    
    public static void holdPosition(DcMotor motor, int encoderValue, double power)
    {
        MotorController controller;
        if ((controller = controllers.get(motor)) == null)
        {
            controller = new MotorController(motor);
            controllers.put(motor, controller);
        }
        controller.hold(encoderValue);
    }
    
    public static void holdUntilComplete(DcMotor motor, int encoderValue, double power, Config conf)
    {
        MotorController controller;
        if ((controller = controllers.get(motor)) == null || controller.closed())
        {
            controller = new MotorController(motor, conf);
            controllers.put(motor, controller);
        }
        controller.setPower(power);
        controller.startRunToPosition(encoderValue);
    }
    
    @Deprecated
    public static void moveToPosition(DcMotor motor, int encoderValue, double power) throws InterruptedException
    {
        holdPosition(motor, encoderValue, power);
        Telemetry.Item[] items = new Telemetry.Item[2];
        while (motor.isBusy())
        {
            items[0] = BaseAutonomous.instance().telemetry.addData("Encoder count", motor.getCurrentPosition());
            items[1] = BaseAutonomous.instance().telemetry.addData("Target position", encoderValue);
            BaseAutonomous.instance().telemetry.update();
            Thread.sleep(10);
        }
        BaseAutonomous.instance().telemetry.removeItem(items[0]);
        BaseAutonomous.instance().telemetry.removeItem(items[1]);
        stopHoldingPosition(motor);
    }
    
    public static void stopHoldingPosition(DcMotor motor)
    {
        MotorController controller;
        if ((controller = controllers.get(motor)) == null) return;
        controllers.remove(controller).close();
    }
}
