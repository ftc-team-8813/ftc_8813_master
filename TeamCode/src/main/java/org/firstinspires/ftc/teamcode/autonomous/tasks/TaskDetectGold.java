package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;

public class TaskDetectGold implements Task
{
    private final ShapeGoldDetector detector;

    public TaskDetectGold(ShapeGoldDetector detector)
    {
        this.detector = detector;
    }

    @Override
    public void runTask() throws InterruptedException
    {
        Telemetry telemetry = BaseAutonomous.instance().telemetry;
        Robot robot = Robot.instance();
        DcMotor left = robot.leftFront;
        DcMotor right = robot.rightFront;
        double speed = 0.1;

        while (!detector.goldSeen())
        {
            int i = 0;
            {
                left.setPower(speed);
                right.setPower(-speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() < i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(750);
                if (detector.goldSeen()) break;
            }

            i = 25;
            {
                left.setPower(speed);
                right.setPower(-speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() < i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(750);
                if (detector.goldSeen()) break;
            }

            i = -25;
            {
                left.setPower(-speed);
                right.setPower(speed);
                while (!detector.goldSeen() && -robot.imu.getHeading() > i)
                {
                    robot.imu.update();
                    Thread.sleep(1);
                }
                left.setPower(0);
                right.setPower(0);
                Thread.sleep(750);
                if (detector.goldSeen()) break;
            }
        }

    }
}
