package org.firstinspires.ftc.teamcode.autonomous.util.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.sensors.IMU;

import java.io.File;

/**
 * Created by aidan on 3/24/18.
 */
@Autonomous(name = "IMU Recalibration", group = "util")
public class IMURecalibrator extends BaseAutonomous
{
    
    private Logger log = new Logger("IMU Recalibrator");
    private IMU imu;
    
    @Override
    public void initialize()
    {
        log.d("Deleting old calibration file");
        File f = new File(Config.storageDir + "imu_calibration.json");
        if (!f.delete())
        {
            log.w("Unable to delete calibration");
        }
        log.d("Initializing IMU");
        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.initialize(telemetry);
    }
    
    @Override
    public void run() throws InterruptedException
    {
        log.d("Starting IMU");
        imu.start();
    }
    
    @Override
    public void finish()
    {
        imu.stop();
    }
}
