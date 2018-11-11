package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.util.sensors.vision.GoldDetector;
import org.opencv.core.Mat;

@Autonomous(name="OpenCV test")
public class OpenCVTest extends BaseAutonomous
{
    
    @Override
    public void run() throws InterruptedException
    {
        CameraStream stream = getCameraStream();
        GoldDetector detector = new GoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);

        while (opModeIsActive())
        {
            telemetry.addData("Gold on-screen: ", detector.goldSeen());
            if (detector.getLocation() != null)
            {
                telemetry.addData("Location", detector.getLocation());
            }
            telemetry.update();
        }
    }
}
