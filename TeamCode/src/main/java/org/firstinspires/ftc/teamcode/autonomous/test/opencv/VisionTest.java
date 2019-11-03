package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.opencv.core.Rect;

@Autonomous(name="Vision Test")
public class VisionTest extends BaseAutonomous
{
    private SkystoneDetector detector;
    @Override
    public void initialize() throws InterruptedException
    {
        CameraStream stream = getCameraStream();
        detector = new SkystoneDetector();
        stream.addListener(detector);
        stream.addModifier(detector);
    }
    
    @Override
    public void run() throws InterruptedException
    {
        while (opModeIsActive())
        {
            telemetry.clearAll();
            telemetry.addData("Detected", detector.found());
            if (detector.found())
            {
                Rect area = detector.getArea();
                int center_x = area.x + area.width/2;
                int center_y = area.y + area.height/2;
                telemetry.addData("Position", "(%d, %d)", center_x, center_y);
                telemetry.addData("Size", "(%d, %d)", area.width, area.height);
            }
            telemetry.update();
            Thread.sleep(100);
        }
    }
}
