package org.firstinspires.ftc.teamcode.autonomous.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * Created by aidan on 3/19/18.
 */

public class IMUMotorController extends MotorController {

    private static class IMUParallelController extends ParallelController {

        private IMU imu;

        IMUParallelController(DcMotor motor, Runnable atTarget, IMU imu, int sse) {
            super(motor, atTarget, sse, false);
            this.imu = imu;
        }

        @Override
        int getCurrentPosition() {
            return (int) Utils.scaleRange(imu.getHeading(), 0, 360, 0, 12455);
        }
    }

    public IMUMotorController(DcMotor motor, Config conf, IMU imu, Runnable atTarget) {
        super(new IMUParallelController(motor, atTarget, imu,
                        conf.getInt("steady_state_error_imu", 0)), conf, "pid_constants_imu");
    }

    public IMUMotorController(DcMotor motor, IMU imu, Runnable atTarget) {
        this(motor, BaseAutonomous.instantated() ? BaseAutonomous.instance().config : null, imu,
                atTarget);
    }

    public IMUMotorController(DcMotor motor, IMU imu, Config conf) {
        this(motor, conf, imu, null);
    }

    public IMUMotorController(DcMotor motor, IMU imu) {
        this(motor, imu, (Runnable)null);
    }
}
