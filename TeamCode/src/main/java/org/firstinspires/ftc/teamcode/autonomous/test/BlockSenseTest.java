package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    public void initialize() throws InterruptedException
    {
        getCameraStream();
        Robot.instance().centerRange.disable();
        Robot.instance().drivetrain.enableFieldCentric();
        Robot.instance().drivetrain.enableAngleCorrection();
        Robot.instance().drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        Drivetrain drivetrain = robot.drivetrain;
    
        WebcamStream stream = (WebcamStream)getCameraStream();
        SkystoneDetector detector = new SkystoneDetector();
        stream.addListener(detector);
        stream.addModifier(detector);
        double fwdStart = robot.odometry.getForwardDistance();
        
        while (opModeIsActive())
        {
            if (detector.found())
            {
                double xError = detector.getCenter().x - 320;
                drivetrain.drive(0.5, xError * 0.001, 0);
            }
            else
            {
                drivetrain.stop();
            }
            if (robot.odometry.getForwardDistance() - fwdStart > 70) break;
            Thread.sleep(5);
        }
        drivetrain.stop();
    }
}
