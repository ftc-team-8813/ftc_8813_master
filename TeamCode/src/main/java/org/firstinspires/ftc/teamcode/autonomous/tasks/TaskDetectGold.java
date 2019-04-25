package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.ShapeGoldDetector;
import org.firstinspires.ftc.teamcode.common.util.Profiler;

public class TaskDetectGold implements Task
{
    private final ShapeGoldDetector detector;
    private Profiler profiler;

    public TaskDetectGold(ShapeGoldDetector detector, Profiler profiler)
    {
        this.detector = detector;
        this.profiler = profiler;
    }

    @Override
    public void runTask() throws InterruptedException
    {
        Telemetry telemetry = BaseAutonomous.instance().telemetry;
        Robot robot = Robot.instance();
        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        double speed = 0.2;

        int counter = 0;
        while (!detector.goldSeen())
        {
            profiler.start("iter" + counter);
            int i = 0;
            {
                profiler.start("straight");
                left.setPower(speed);
                right.setPower(-speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() < i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(250);
                profiler.end();
                if (detector.goldSeen())
                {
                    profiler.end();
                    break;
                }

            }

            i = 25;
            {
                profiler.start("right");
                left.setPower(speed);
                right.setPower(-speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() < i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(500);
                profiler.end();
                if (detector.goldSeen())
                {
                    profiler.end();
                    break;
                }
            }

            i = -25;
            {
                profiler.start("left");
                left.setPower(-speed);
                right.setPower(speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() > i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(500);
                profiler.end();
                if (detector.goldSeen())
                {
                    profiler.end();
                    break;
                }
            }
            profiler.end();
            counter++;
        }
    }
}
