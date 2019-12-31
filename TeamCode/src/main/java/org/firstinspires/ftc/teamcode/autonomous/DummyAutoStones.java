package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;


@Autonomous(name="DummyAutoStones")
public class DummyAutoStones extends BaseAutonomous
{
    private WebcamStream externalCamera;
    
    public void initialize(){

        Robot robot = Robot.instance();
        robot.intakelinkage.moveLinkageIn();
        robot.newarm.resetArm();
        robot.claw.openClaw();
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
        
        externalCamera = new WebcamStream();
        SkystoneDetector skystone = new SkystoneDetector();
        externalCamera.addListener(skystone);
        externalCamera.addModifier(skystone);
        String filename = "videos/stones_" + Utils.getTimestamp() + ".avi";
        externalCamera.addListener(new Vlogger(filename, 640, 480, 15), 1001);

        robot.drivetrain.move(0.3, 0, 0, tickstoInches(22));
        Thread.sleep(2000);
        robot.drivetrain.stop();

        while(!skystone.found() || skystone.getArea().width<300){
            robot.drivetrain.move(0, 0.2, 0, tickstoInches(7.7));
            Thread.sleep(1000);
            robot.drivetrain.stop();

            robot.drivetrain.move(0, 0, 0.1, 1);
            Thread.sleep(1000);
            robot.drivetrain.stop();
        }

        robot.drivetrain.move(0, 0.3, 0, tickstoInches(5.25));
        robot.drivetrain.stop();

        robot.newarm.moveArmEnc(0.4, 900);
        Thread.sleep(1000);
        robot.newarm.moveArm(0);

        robot.claw.closeClaw();
        Thread.sleep(100);

        robot.slide.raiseLiftEnc(tickstoInches(2));
        Thread.sleep(1000);

        robot.drivetrain.move(-0.4, 0, 0, tickstoInches(23));

        robot.drivetrain.move(0, -0.8, 0, tickstoInches(40));

        robot.claw.openClaw();

        robot.drivetrain.move(0, 0.6, 0, tickstoInches(10));

        robot.drivetrain.move(0, 0.6, 0, tickstoInches(30));
    }
    
    public void finish()
    {
        externalCamera.stop();
    }

    public int tickstoInches(double dist){
        final double CIRCUMFERENCE = 3.14*(100/25.4);
        double ticks = (dist/CIRCUMFERENCE)*537.6;
        return (int) ticks;
    }
}
