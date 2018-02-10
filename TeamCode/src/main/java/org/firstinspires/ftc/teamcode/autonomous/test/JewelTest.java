package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.MainAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskScoreJewel;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;

/**
 * Created by aidan on 2/9/18.
 */
@Autonomous(name="Jewel Tester", group="test")
public class JewelTest extends BaseAutonomous {

    @Override
    public void run() throws InterruptedException {
        Servo colorArm = hardwareMap.servo.get("s5");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DcMotor motor = hardwareMap.dcMotor.get("base");
        MotorController base = new MotorController(motor);
        tasks.add(new TaskScoreJewel(2, base, colorArm, colorSensor));
    }
}
