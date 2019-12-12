package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.vision.CameraStream;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;

@Autonomous(name="Stones")
public class StoneAuto extends BaseAutonomous
{
    private Robot robot;
    private CameraStream stream;
    private SkystoneDetector detector;
    
    @Override
    public void initialize() throws InterruptedException
    {
        Robot.instance().intakelinkage.moveLinkageIn();
        stream = getCameraStream();
        Robot.instance().imu.setImmediateStart(true);
        Robot.instance().imu.initialize();
    }
    
    @Override
    public void run() throws InterruptedException
    {
        robot = Robot.instance();
        robot.intakelinkage.moveLinkageOut();
        Drivetrain drivetrain = robot.drivetrain;
        detector = new SkystoneDetector();
        stream.addListener(detector);
        stream.addModifier(detector);
        stream.addListener(new Vlogger("videos/" + Utils.getTimestamp() + ".avi",  640, 480, 10), 1001);
        
        drivetrain.move(0.3, 0, 0, 800);
        Thread.sleep(100);
        drivetrain.move(0, 0.3, 0, -100);
        Thread.sleep(100);
        drivetrain.move(0, 0, 0.3, -30);
        Thread.sleep(100);
        robot.intake.collectStone(0.4);
        drivetrain.move(0.3, 0, 0, 300);
        Thread.sleep(500);
        robot.intake.stopIntake();
        drivetrain.move(0.3, 0, 0, -200);
        Thread.sleep(200);
        drivetrain.move(0, 0, 0.4, 30);
        Thread.sleep(100);
        drivetrain.move(0.3, 0, 0, -200);
        robot.intake.collectStone(0.4);
        Thread.sleep(500);
        robot.intake.stopIntake();
        drivetrain.move(0, 0, 0.4, 60);
        drivetrain.move(0.3, 0, 0, 1000);
        robot.intake.collectStone(-0.3);
        Thread.sleep(500);
        robot.intake.stopIntake();
        drivetrain.move(0.3, 0, 0, -800);
        
    }
    
    @Override
    public void finish() throws InterruptedException
    {
    
    }
}
