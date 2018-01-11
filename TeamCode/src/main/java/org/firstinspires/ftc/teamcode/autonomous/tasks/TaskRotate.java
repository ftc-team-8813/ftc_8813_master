package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;

/**
 * TaskRotate - Rotate a motor to a specific position
 */

public class TaskRotate implements Task {

    private MotorController motor;
    private int angle;

    public TaskRotate(MotorController motor, int angle) {
        this.motor = motor;
        this.angle = angle;
    }

    @Override
    public void runTask() throws InterruptedException {
        motor.runToPosition(angle);
    }

}
