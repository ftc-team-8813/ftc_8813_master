package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.ShapeGoldDetector;
import org.opencv.android.OpenCVLoader;

@Autonomous(name="Webcam Stream Test")
@Disabled
public class WebcamStreamTest extends LinearOpMode
{

    static
    {
        if (!OpenCVLoader.initDebug())
        {
            System.exit(0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        WebcamStream stream = new WebcamStream();
        waitForStart();
        while (opModeIsActive()) Thread.sleep(5);
        stream.stop();
    }
}
