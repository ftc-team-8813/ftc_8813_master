package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.sensors.vision.Webcam;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Stones")
public class StoneAuto extends BaseAutonomous
{
    private Robot robot;
    private WebcamStream stream;
    private SkystoneDetector detector;
    private Drivetrain drivetrain;
    
    @Override
    public void initialize() throws InterruptedException
    {
        Robot.instance().intakelinkage.moveLinkageIn();
        CameraStream rawStream = getCameraStream();
        if (rawStream instanceof WebcamStream)
        {
            stream = (WebcamStream)rawStream;
        }
        else
        {
            throw new IllegalStateException("Only webcam is supported. Is it plugged in?");
        }
        ExposureControl control = stream.getInternalCamera().getControl(ExposureControl.class);
        // control.setMode(ExposureControl.Mode.Manual);
        Robot.instance().imu.setImmediateStart(true);
        Robot.instance().imu.initialize();
    }
    
    private void pickBlock(int turn, int strafe_dist, int fwd_dist, int back_dist) throws InterruptedException
    {
        int off = 0;
        if (detector.found())
        {
            off = detector.getArea().x + detector.getArea().width/2 - 320;
        }
        drivetrain.move(0, 0.4, 0, -strafe_dist + (int)(off*0.67));
        Thread.sleep(100);
        drivetrain.move(0, 0, 0.4, -turn);
        Thread.sleep(100);
        robot.intake.collectStone(0.4);
        drivetrain.move(0.4, 0, 0, fwd_dist);
        Thread.sleep(400);
        robot.intake.stopIntake();
        drivetrain.move(0, 0, 0.4, turn);
        Thread.sleep(100);
        drivetrain.move(0.45, 0, 0, -back_dist);
        robot.intake.collectStone(0.4);
        Thread.sleep(150);
        robot.intake.stopIntake();
        // drivetrain.move(0, 0, 0.6, -((int)robot.imu.getHeading()) + 50);
    }
    
    private int senseBlock() throws InterruptedException
    {
        stream.getInternalCamera().getControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
        int startPos = drivetrain.rightBack.getCurrentPosition();
        drivetrain.drive(0, 0.3, 0);
        final double SPD_NORMAL = 0.3;
        final double SPD_SLOW   = 0.1;
        final double SPD_EXSLOW = 0.05;
    
        while (drivetrain.rightBack.getCurrentPosition() - startPos < 2200)
        {
            if (detector.found())
            {
                if (detector.getArea().width > 300)
                {
                    break;
                }
                else if (detector.getArea().width > 200)
                {
                    drivetrain.drive(SPD_SLOW, 0, 0);
                }
                else if (detector.getArea().width > 80)
                {
                    drivetrain.drive(0, SPD_SLOW, 0);
                }
                else
                {
                    drivetrain.drive(0, SPD_NORMAL, 0);
                }
            }
            else
            {
                drivetrain.drive(0, SPD_NORMAL, 0);
            }
            Thread.sleep(1);
        }
    
        drivetrain.stop();
        Thread.sleep(100);
        return drivetrain.rightBack.getCurrentPosition() - startPos;
    }
    
    @Override
    public void run() throws InterruptedException
    {
        /////////////////
        // Initialization
        // Robot
        robot = Robot.instance();
        robot.intakelinkage.moveLinkageOut();
        // Drivetrain
        drivetrain = robot.drivetrain;
        // Vision
        detector = new SkystoneDetector();
        stream.addListener(detector);
        stream.addModifier(detector);
        stream.addListener(new Vlogger("videos/" + Utils.getTimestamp() + ".avi",  640, 480, 10), 1001);
        GlobalDataLogger.instance().addChannel("Skystone Width", () -> "" + (detector.found() ? detector.getArea().width : 0));
        
        // Initial forward
        drivetrain.move(0.4, 0, 0, 650);
        Thread.sleep(100);
        
        // Sense block
        int senseDist = senseBlock();
        drivetrain.move(0, 0, 0.1, -(int)robot.imu.getHeading());
        
        pickBlock(14, 160, 350, 1500);
    
        drivetrain.move(0, 0, 0.2, -(int)robot.imu.getHeading() - 2); // Medium correction
        // drivetrain.move(0, 0, 0.2, -(int)robot.imu.getHeading()); // Another fine correction
        Thread.sleep(100);
        
        drivetrain.move(0, 0.8, 0, -(senseDist + 850));
        Thread.sleep(150);
        robot.intake.collectStone(-0.3);
        Thread.sleep(500);
        robot.intake.stopIntake();
        drivetrain.move(0, 0, 0.1, -(int)robot.imu.getHeading() + 2); // Fine correction
        drivetrain.move(0, 0.85, 0, 850 + senseDist);
        Thread.sleep(200);
        drivetrain.move(0.6, 0, 0, 550);
        Thread.sleep(500);
        drivetrain.move(0, 0.5, 0, 700);
        // drivetrain.move(0, 0, 0.1, -(int)robot.imu.getHeading());
        
        senseBlock();
        // drivetrain.move(0, 0.5, 0, 25);
        drivetrain.move(0, 0, 0.15, -(int)robot.imu.getHeading());
        
        // drivetrain.move(0.6, 0, 0, -1);
        pickBlock(12, 220, 400, 700);
        
        drivetrain.move(0, 1, 0, -1900 - senseDist);
        robot.intake.collectStone(-0.3);
        Thread.sleep(400);
        robot.intake.stopIntake();
        drivetrain.move(0, 1, 0, 300);
    }
    
    @Override
    public void finish() throws InterruptedException
    {
    
    }
}
