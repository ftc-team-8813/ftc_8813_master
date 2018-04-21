package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.util.IMUMotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.Config;

/**
 * Created by aidan on 3/22/18.
 */
@TeleOp(name = "PID Test", group="test")
public class PIDTest extends OpMode {

    private MotorController controller;

    @Override
    public void init() {
        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.initialize(telemetry);
        imu.start();
        controller = new IMUMotorController(
                hardwareMap.dcMotor.get("base"), imu, new Config("config.properties"));
    }

    @Override
    public void loop() {
        controller.hold((int)(controller.getTargetPosition() + gamepad1.left_stick_y * 10));
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            return;
        }
    }

    @Override
    public void stop() {
        controller.close();
    }
}
