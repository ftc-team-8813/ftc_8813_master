package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.teamcode.autonomous.vision.SkystoneDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.sensors.vision.WebcamStream;


@Autonomous(name="DummyAutoStones")
public class DummyAutoStones extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        WebcamStream externalCamera = new WebcamStream();
        SkystoneDetector skystone = new SkystoneDetector();
        robot.arm.openClaw();
        Thread.sleep(1000);

        robot.drivetrain.move(0.2, 0, 0, 935);
        Thread.sleep(1000);
        robot.drivetrain.stop();

        robot.arm.extend(0.53);
        Thread.sleep(1000);

        robot.arm.closeClaw();
        Thread.sleep(1000);

        robot.drivetrain.move(-0.2, 0, 0, tickstoInches(8));
        Thread.sleep(1000);

        robot.drivetrain.move(0, -0.3, 0, tickstoInches(30));
/*        while(!skystone.found()){
            robot.drivetrain.move(0, 0.2, 0, tickstoInches(8));
            Thread.sleep(1000);
            enumerator += 1;
        }*/
    }

    public int tickstoInches(double dist){
        return (int) ((dist/(3.14*4))*537.6);
    }
}
