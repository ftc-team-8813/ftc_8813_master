package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.sensors.CurrentSensor;

@TeleOp(name="Current Sense Test")
public class CurrentSenseTest extends OpMode {
    private CurrentSensor sensor;
    private Servo claw;
    @Override
    public void init() {
        sensor = new CurrentSensor(hardwareMap.analogInput.get("clawSensor"));
        claw = hardwareMap.servo.get("s3");
    }

    @Override
    public void loop() {
        claw.setPosition((gamepad1.left_stick_y + 1) / 2);
        telemetry.addData("Claw position", claw.getPosition());
        telemetry.addData("Analog input voltage", sensor.getInputVoltage());
        telemetry.addData("Claw current draw", sensor.getCurrent());
    }
}
