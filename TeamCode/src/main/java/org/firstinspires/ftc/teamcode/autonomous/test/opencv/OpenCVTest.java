package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.opencv.core.Mat;

public class OpenCVTest extends BaseAutonomous
{
    
    @Override
    public void run() throws InterruptedException
    {
        CameraStream stream = getCameraStream();
        stream.setModifier(new CameraStream.OutputModifier()
        {
            @Override
            public Mat process(Mat rgba)
            {
                return null;
            }
        });
    }
    
}
