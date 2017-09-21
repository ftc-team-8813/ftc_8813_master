package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 *
 */

public class TaskMoveMotor implements Task {
    private DcMotor motor;
    public TaskMoveMotor(DcMotor motor) {this.motor = motor;}

    @Override
    public void runTask() throws InterruptedException {
        motor.setPower(1.0);
        Thread.sleep(1000);
        motor.setPower(0.0);
    }
}