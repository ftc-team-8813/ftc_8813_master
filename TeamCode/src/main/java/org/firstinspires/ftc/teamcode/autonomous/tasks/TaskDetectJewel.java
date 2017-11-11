package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * TaskDetectJewel - Detect whether the color sensor sees red or blue and return which one it saw
 */

public class TaskDetectJewel implements Task {

    public static final int RED     = 1;
    public static final int BLUE    = 2;
    public static final int UNKNOWN = 0;

    private ColorSensor sensor;
    private int output;
    private int[] detected;

    public int getReading() {
        return output;
    }

    public int[] getDetectedColor() {
        return detected;
    }

    public TaskDetectJewel(ColorSensor sensor) {
        this.sensor = sensor;
    }


    @Override
    public void runTask() throws InterruptedException {
        sensor.enableLed(true);
        if (sensor.blue() > sensor.red() && sensor.blue() > sensor.green()) {
            output = BLUE;
        } else if (sensor.red() > sensor.green() && sensor.red() > sensor.blue()) {
            output = RED;
        } else {
            output = UNKNOWN;
        }
        detected = new int[] {sensor.red(), sensor.green(), sensor.blue()};
        sensor.enableLed(false);
    }
}
