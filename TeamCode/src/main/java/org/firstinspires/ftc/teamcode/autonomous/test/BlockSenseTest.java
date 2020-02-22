package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;

@Autonomous(name="Block Sense Test")
public class BlockSenseTest extends BaseAutonomous
{
    
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        Drivetrain drivetrain = robot.drivetrain;
    
        WebcamStream stream = (WebcamStream)getCameraStream();
        SkystoneDetector detector = new SkystoneDetector();
        stream.addListener(detector);
        stream.addModifier(detector);
        
        while (opModeIsActive())
        {
            if (detector.found())
            {
                double xError = detector.getCenter().x - 320;
                drivetrain.drive(0.1, xError * 0.005, 0);
                Thread.sleep(5);
            }
            else
            {
                drivetrain.stop();
            }
        }
    }
}
