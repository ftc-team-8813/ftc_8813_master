package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;

@Autonomous(name="Color Test")
public class ColorCounter extends BaseAutonomous
{
    private ColorSensor sensor;
    
    @Override
    public void run() throws InterruptedException
    {
        tasks.add(new Task()
        {
            @Override
            public void runTask() throws InterruptedException
            {
                sensor = hardwareMap.colorSensor.get("color");
                telemetry.addData("Color sensor address", sensor.getI2cAddress());
                while (opModeIsActive())
                {
                    float[] hsv = new float[3];
                    Color.colorToHSV(Color.rgb(sensor.red(), sensor.green(), sensor.blue()), hsv);
                    telemetry.addData("Hue", hsv[0]);
                    telemetry.addData("Saturation", hsv[1]);
                    telemetry.addData("Value", hsv[2]);
                }
            }
        });
    }
}
