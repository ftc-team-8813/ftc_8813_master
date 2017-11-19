package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Servo initialization for autonomous
 */

public class ServoInit {
    public static void initServos(Config config, String property, Servo... servos) {
        double[] values = config.getDoubleArray(property);
        if (values == null)
            return;
        int i = 0;
        for (double position : values) {
            if (i >= servos.length)
                return;
            if (servos[i] != null) servos[i].setPosition(position);
        }
    }
}
