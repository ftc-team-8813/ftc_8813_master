package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;

/**
 * Created by anna1 on 9/20/2017. The TaskReadSensor will read the value of the color sensor and send that to
 * the driver station phone
 * */

public class TaskReadSensor implements Task {
    private ColorSensor sensor;
    public TaskReadSensor(ColorSensor sensor){
        this.sensor=sensor;
    }
    @Override
    public void runTask() throws InterruptedException {
        while(true){
            String Display=sensor.red() + ", " + sensor.green() + ", " + sensor.blue();
            BaseAutonomous.instance().telemetry.addData("Sensor Value", Display);
        }
    }
}
