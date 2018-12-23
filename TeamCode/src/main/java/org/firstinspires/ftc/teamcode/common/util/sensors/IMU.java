package org.firstinspires.ftc.teamcode.common.util.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

/**
 * Wrapper class for the BNO055 IMU to automatically set up and calibrate the IMU.
 */

public class IMU
{
    
    //Modes
    private static final int PRE_INIT = 0;
    private static final int INITIALIZED = 1;
    private static final int SETUP = 2;
    private static final int STARTED = 3;
    
    //The IMU
    private BNO055IMU imu;
    //Its parameters
    private BNO055IMU.Parameters params;
    //The logger
    private Logger log;
    private boolean autoCalibrating;
    private Thread worker;
    private float heading, roll, pitch;
    private int status = PRE_INIT;
    
    public IMU(BNO055IMU imu)
    {
        this.imu = imu;
        log = new Logger("IMU Wrapper");
    }
    
    public void initialize(Telemetry telemetry)
    {
        //Initialize telemetry
        TelemetryWrapper.init(telemetry, 5);
        //Set up
        params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.mode = BNO055IMU.SensorMode.IMU;
        try
        {
            File f = new File(Config.storageDir + "imu_calibration.json");
            if (f.exists())
            {
                TelemetryWrapper.setLine(0, "Reading calibration file");
                //Read the entire file into a String so we can then deserialize it
                FileReader reader = new FileReader(f);
                char[] buffer = new char[1024];
                int len;
                StringBuilder sb = new StringBuilder();
                while ((len = reader.read(buffer)) > 0)
                {
                    if (len == 1024)
                    {
                        sb.append(buffer); //Don't copy the array if it's full
                    } else
                    {
                        sb.append(Arrays.copyOfRange(buffer, 0, len)); //Get the non-empty chunk
                    }
                }
                reader.close();
                params.calibrationData = BNO055IMU.CalibrationData.deserialize(sb.toString());
            } else
            {
                log.w("No calibration file; auto calibration will be used instead.");
                autoCalibrating = true;
                
            }
        } catch (IOException e)
        {
            log.e("Unable to read calibration file; auto calibration will be used instead.");
            autoCalibrating = true;
        }
        params.loggingEnabled = false;
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        if (autoCalibrating)
        {
            TelemetryWrapper.setLine(0, "Calibration required. Please set the robot on a flat " +
                    "surface");
        }
        status = INITIALIZED;
    }
    
    public void start()
    {
        start(false);
    }
    
    public void start(final boolean inRadians)
    {
        if (status < INITIALIZED)
        {
            log.f("start() called before initialize()!");
            throw new IllegalStateException("start() called before initialize()!");
        }
        if (status == STARTED)
        {
            log.d("Trying to start IMU even though it is already running");
            return;
        }
        if (status != SETUP)
        {
            TelemetryWrapper.setLine(0, "Initializing IMU");
            imu.initialize(params);
            while (autoCalibrating && !Thread.interrupted())
            {
                if (autoCalibrating)
                {
                    TelemetryWrapper.setLine(0, "Calibrating... Please do not disturb the " +
                            "robot!");
                    int progress = (imu.getCalibrationStatus().calibrationStatus >> 4) & 3;
                    TelemetryWrapper.setLine(1, "Progress: " + progress + "/3");
                    TelemetryWrapper.setLine(2, "Status: " + imu.getSystemStatus().toString());
                    if (progress == 3)
                    {
                        TelemetryWrapper.setLine(1, "Progress: Saving calibration");
                        autoCalibrating = false;
                        File f = new File(Config.storageDir + "imu_calibration.json");
                        try
                        {
                            String data = imu.readCalibrationData().serialize();
                            FileWriter w = new FileWriter(f);
                            w.write(data);
                            w.close();
                        } catch (IOException e)
                        {
                            log.e("Unable to write calibration data");
                        }
                    }
                }
                try
                {
                    Thread.sleep(20);
                } catch (InterruptedException e)
                {
                    break;
                }
            }
            status = SETUP;
            TelemetryWrapper.clear();
        }
        worker = new Thread(new Runnable()
        {
            
            private float lastAngle;
            private int revolutions;
            
            @Override
            public void run()
            {
                while (!Thread.interrupted())
                {
                    Orientation o = imu.getAngularOrientation();
                    float h = o.firstAngle;
                    roll = o.secondAngle;
                    pitch = o.thirdAngle;
                    if (inRadians)
                    {
                        h = (float) Math.toDegrees(h);
                        roll = (float) Math.toDegrees(roll);
                        pitch = (float) Math.toDegrees(pitch);
                    }
                    float delta = h - lastAngle;
                    if (delta < -300)
                    {
                        //Looped past 180 to -179
                        revolutions++;
                    } else if (delta > 300)
                    {
                        //Looped past -179 to 180
                        revolutions--;
                    }
                    lastAngle = h;
                    heading = h + 360 * revolutions;
                    try
                    {
                        Thread.sleep(10);
                    } catch (InterruptedException e)
                    {
                        break;
                    }
                }
            }
        });
        worker.setDaemon(true);
        worker.start();
        status = STARTED;
    }
    
    public float getHeading()
    {
        return heading;
    }
    
    public float getRoll()
    {
        return roll;
    }
    
    public float getPitch()
    {
        return pitch;
    }
    
    public void stop()
    {
        if (status != STARTED)
        {
            log.d("Trying to stop IMU even though it is not running");
            return;
        }
        worker.interrupt();
        status = SETUP;
    }
}
