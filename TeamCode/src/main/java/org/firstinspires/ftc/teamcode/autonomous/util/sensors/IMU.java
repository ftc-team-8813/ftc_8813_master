package org.firstinspires.ftc.teamcode.autonomous.util.sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

/**
 * Created by aidan on 3/20/18.
 */

public class IMU {

    private BNO055IMU imu;
    private BNO055IMU.Parameters params;
    private Logger log;
    private boolean autoCalibrating;
    private Thread worker;
    private float heading, roll, pitch;

    public IMU(BNO055IMU imu) {
        this.imu = imu;
        log = new Logger("IMU Wrapper");
    }

    public void initialize(Telemetry telemetry) {
        //Initialize telemetry
        TelemetryWrapper.init(telemetry, 5);
        //Set up
        params = new BNO055IMU.Parameters();
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.mode                = BNO055IMU.SensorMode.IMU;
        try {
            File f = new File(Config.storageDir + "imu_calibration.json");
            if (f.exists()) {
                TelemetryWrapper.setLine(0, "Reading calibration file");
                //Read the entire file into a String so we can then deserialize it
                FileReader reader = new FileReader(f);
                char[] buffer = new char[1024];
                int len;
                StringBuilder sb = new StringBuilder();
                while ((len = reader.read(buffer)) > 0) {
                    if (len == 1024) {
                        sb.append(buffer); //Don't copy the array if it's full
                    } else {
                        sb.append(Arrays.copyOfRange(buffer, 0, len)); //Get the non-empty chunk
                    }
                }
                reader.close();
                params.calibrationData = BNO055IMU.CalibrationData.deserialize(sb.toString());
            } else {
                log.w("No calibration file; auto calibration will be used instead.");
                autoCalibrating = true;

            }
        } catch (IOException e) {
            log.e("Unable to read calibration file; auto calibration will be used instead.");
            autoCalibrating = true;
        }
        params.loggingEnabled      = false;
        params.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        if (autoCalibrating) {
            TelemetryWrapper.setLine(0, "Calibration required. Set the robot on a flat surface " +
                    "before pressing play.");
        }
    }

    public void start() {
        TelemetryWrapper.setLine(0, "Initializing IMU");
        imu.initialize(params);
        while (autoCalibrating && !Thread.interrupted()) {
            if (autoCalibrating) {
                TelemetryWrapper.setLine(0, "Calibrating... Please do not disturb the " +
                        "robot!");
                int progress = (imu.getCalibrationStatus().calibrationStatus >> 4) & 3;
                TelemetryWrapper.setLine(1, "Progress: " + progress + "/3");
                TelemetryWrapper.setLine(2, "Status: " + imu.getSystemStatus().toString());
                if (progress == 3) {
                    TelemetryWrapper.setLine(1, "Progress: Saving calibration");
                    autoCalibrating = false;
                    File f = new File(Config.storageDir + "imu_calibration.json");
                    try {
                        String data = imu.readCalibrationData().serialize();
                        FileWriter w = new FileWriter(f);
                        w.write(data);
                        w.close();
                    } catch (IOException e) {
                        log.e("Unable to write calibration data");
                    }
                }
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }
        }
        TelemetryWrapper.clear();
        worker = new Thread(new Runnable() {

            private float lastAngle;
            private int revolutions;

            @Override
            public void run() {
                while (!Thread.interrupted()) {
                    Orientation o = imu.getAngularOrientation();
                    float h = o.firstAngle;
                    roll    = o.secondAngle;
                    pitch   = o.thirdAngle;
                    float delta = h - lastAngle;
                    if (delta < -300) {
                        //Looped past 180 to -179
                        revolutions++;
                    } else if (delta > 300) {
                        //Looped past -179 to 180
                        revolutions--;
                    }
                    lastAngle = h;
                    heading = h + 360 * revolutions;
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        break;
                    }
                }
            }
        });
        worker.setDaemon(true);
        worker.start();
    }

    public float getHeading() {
        return heading;
    }

    public float getRoll() {
        return roll;
    }

    public float getPitch() {
        return pitch;
    }

    public void stop() {
        worker.interrupt();
    }
}
