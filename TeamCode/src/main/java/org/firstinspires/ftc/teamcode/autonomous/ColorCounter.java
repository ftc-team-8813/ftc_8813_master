package org.firstinspires.ftc.teamcode.autonomous;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autonomous.tasks.Task;
import org.firstinspires.ftc.teamcode.util.ColorRange;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.opencv.android.JavaCameraView;

@Autonomous(name="Color Test")
public class ColorCounter extends BaseAutonomous
{
    private ColorSensor sensor;
    private volatile boolean uiRunning;
    private Logger log = new Logger("Color Sense");
    private Activity a;
    
    @Override
    public void run() throws InterruptedException
    {
        a = AppUtil.getInstance().getActivity();
        FtcRobotControllerActivity rc = (FtcRobotControllerActivity) a;
        final LinearLayout cameraLayout = rc.cameraMonitorLayout;
    
        while (uiRunning)
        {
            try
            {
                Thread.sleep(0, 100);
            } catch (InterruptedException e) {
                log.w("Ignoring interrupt");
            }
        }
    
        sensor = hardwareMap.colorSensor.get("color");
        telemetry.addData("Color sensor address", sensor.getI2cAddress());
        while (opModeIsActive())
        {
            float[] hsv = new float[3];
            NormalizedRGBA rgba = ((AMSColorSensor)sensor).getNormalizedColors();
            final int r = (int)(Math.min(255, Math.log(rgba.red+1)*2000));
            final int g = (int)(Math.min(255, Math.log(rgba.green+1)*2500));
            final int b = (int)(Math.min(255, Math.log(rgba.blue+1)*2500));
            Color.colorToHSV(Color.rgb(r, g, b), hsv);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);
            telemetry.addData("Red", r);
            telemetry.addData("Green", g);
            telemetry.addData("Blue", b);
            telemetry.addData("Color", ColorRange.toString(ColorRange.getColor(r, g, b)));
            telemetry.update();
            a.runOnUiThread(new Runnable()
            {
                @Override
                public void run()
                {
                    cameraLayout.setBackgroundColor(Color.rgb(r, g, b));
                }
            });
            Thread.sleep(10);
        }
        
        uiRunning = true;
        a.runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                cameraLayout.setBackgroundColor(0);
                uiRunning = false;
            }
        });
        while (uiRunning) ;
    }
}
