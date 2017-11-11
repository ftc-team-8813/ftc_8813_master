package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

/**
 * TaskExtendSlide - Extend the slide out to its limit
 */

public class TaskExtendSlide implements Task {

    public static boolean extended = false;

    private DcMotor ex;

    public TaskExtendSlide(DcMotor extender) {
        extended = false;
        this.ex = extender;
    }

    @Override
    public void runTask() {
        double vel = 0.5;
        ex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ex.setPower(vel);
        int rng = BaseAutonomous.instance().config.getInt("ext_range", 0);
        while (ex.getCurrentPosition() < rng) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {}
        }
        extended = true;
    }
}
