package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Config;

/**
 * Created by aidan on 3/19/18.
 */

public class IMUMotorController extends MotorController {

    private static class IMUParallelController extends ParallelController {

        IMUParallelController(DcMotor motor, Runnable atTarget) {
            super(motor, atTarget);
        }
    }

    public IMUMotorController(DcMotor motor, Config conf, Runnable atTarget) {
        super(new IMUParallelController(motor, atTarget), conf);
    }

    public IMUMotorController(DcMotor motor, Runnable atTarget) {
        super(motor, atTarget);
    }

    public IMUMotorController(DcMotor motor, Config conf) {
        super(motor, conf);
    }

    public IMUMotorController(DcMotor motor) {
        super(motor);
    }
}
