package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ServoAngleFinder - Option to use potentiometers on
 */

public class ServoAngleFinder {

    private static ServoAngleFinder instance;

    public static ServoAngleFinder instance() {
        return instance;
    }

    public static boolean exists() {
        return instance != null;
    }

    private AnalogInput waistPot, shoulderPot, elbowPot;

    private ServoAngleFinder(HardwareMap hardwareMap) {
        if (hardwareMap.analogInput.contains("wp") &&
                hardwareMap.analogInput.contains("sp") &&
                hardwareMap.analogInput.contains("ep")) {
            waistPot = hardwareMap.analogInput.get("wp");
            shoulderPot = hardwareMap.analogInput.get("sp");
            elbowPot = hardwareMap.analogInput.get("ep");
            instance = this;
        } else instance = null;
    }

    public static boolean create(HardwareMap hardwareMap) {
        new ServoAngleFinder(hardwareMap);
        return instance != null;
    }

    public double getWaistAngle() {
        return waistPot.getVoltage()/waistPot.getMaxVoltage();
    }

    public double getShoulderAngle() {
        return shoulderPot.getVoltage()/shoulderPot.getMaxVoltage();
    }

    public double getElbowAngle() {
        return elbowPot.getVoltage()/elbowPot.getMaxVoltage();
    }
}
