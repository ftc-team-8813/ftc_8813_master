package org.firstinspires.ftc.teamcode.autonomous.test;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.security.spec.ECParameterSpec;
import java.util.Arrays;

/**
 * Created by aidan on 3/19/18.
 */
@Autonomous(name="IMU Test", group="test")
public class IMUTest extends BaseAutonomous {

    private Logger log;
    private boolean autoCalibrating;
    private BNO055IMU imu;
    private BNO055IMU.Parameters params;
    private float lastAngle;
    private int revolutions;
    private DcMotor base;
    private int encoderRevolution;

    @Override
    public void initialize() throws InterruptedException {
        log = new Logger("IMU Test");
        base = hardwareMap.dcMotor.get("base");
        //Initialize telemetry
        TelemetryWrapper.init(telemetry, 5);
        //Set up
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        params = new BNO055IMU.Parameters();
        params.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.mode                = BNO055IMU.SensorMode.IMU;
        try {
            File f = new File(Config.storageDir + "imu_calibration.json");
            if (f.exists()) {
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

    @Override
    public void run() throws InterruptedException {


        tasks.add(new Task() {

            @Override
            public void runTask() throws InterruptedException {
                if (autoCalibrating) {
                    params.mode = BNO055IMU.SensorMode.IMU; //Enable fusion mode so we can calibrate
                }
                TelemetryWrapper.setLine(0, "Initializing sensor...");
                imu.initialize(params);
                while (!Thread.interrupted()) {
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
//                            TelemetryWrapper.setLine(1, "Progress: Re-initializing");
//                            params.mode = BNO055IMU.SensorMode.GYRONLY;
//                            imu.initialize(params);
                        }
                    } else {
                        TelemetryWrapper.setLine(0, "IMU Test");
                        TelemetryWrapper.setLine(1, "Status: " + imu.getSystemStatus().toString());
                        Orientation o = imu.getAngularOrientation();
                        float heading = o.firstAngle;
                        float roll    = o.secondAngle;
                        float pitch   = o.thirdAngle;
                        float delta = heading - lastAngle;
                        if (delta < -300) {
                            //Looped past 180 to -179
                            revolutions++;
                        } else if (delta > 300) {
                            //Looped past -179 to 180
                            revolutions--;
                        }

                        float cHeading = (heading + 360 * revolutions);
                        if (Math.abs(cHeading - 360) < 0.05) {
                            encoderRevolution = base.getCurrentPosition();
                        }

                        TelemetryWrapper.setLine(2, "Heading: " + cHeading);
                        TelemetryWrapper.setLine(3, "Encoder: " + (base.getCurrentPosition()));
                        TelemetryWrapper.setLine(4, "360 Deg: " + (encoderRevolution));

                        lastAngle = heading;
                    }
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        break;
                    }
                }
            }
        });
    }
}