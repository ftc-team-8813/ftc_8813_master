package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.ShapeGoldDetector;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.TelemetryWrapper;

/**
 * TaskSample variant that simply lines up and then extends the intake
 */
public class TaskBasicSample implements Task
{
    private ShapeGoldDetector detector;
    private Telemetry telemetry;
    private Logger log;

    public TaskBasicSample(ShapeGoldDetector detector)
    {
        this.detector = detector;
        telemetry = BaseAutonomous.instance().telemetry;
        log = new Logger("TaskBasicSample");
    }

    @Override
    public void runTask() throws InterruptedException
    {
        final double kP = 0.3;
        Robot robot = Robot.instance();
        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        double l = 0, r = 0;
        boolean wasSeen = true;
        while (true)
        {
            if (!detector.goldSeen())
            {
                log.d("Mineral not found -- last seen at %s", detector.getLocation());
                telemetry.addData("Mineral", "Not Seen");
                if (detector.getLocation() != null && detector.getLocation().x < 100 && left.getPower() > 0)
                {
                    telemetry.addData("Mineral lost", "to the left");
                    if (wasSeen)
                    {
                        log.d("Mineral lost to the side; reversing seek direction");
                        l *= -1;
                        r *= -1;
                    }
                }
                else if (detector.getLocation() != null && detector.getLocation().x > 540 && left.getPower() < 0)
                {
                    telemetry.addData("Mineral lost", "to the right");
                    if (wasSeen)
                    {
                        log.d("Mineral lost to the side; reversing seek direction");
                        l *= -1;
                        r *= -1;
                    }
                }
                else
                {
                    l *= 0.8;
                    r *= 0.8;
                }
                wasSeen = false;
            }
            else
            {
                log.d("Mineral found");
                telemetry.addData("Mineral", "Seen");
                double error = -(detector.getLocation().x / 640 - 0.5) * 2;
                error += 0.1; // Adjust for camera placement
                if (Math.abs(error) < 0.03) break;
                // Positive error -> turn RIGHT
                l = -kP * error;
                r = kP * error;
                telemetry.addData("Error", error);
                wasSeen = true;
            }

            left.setPower(l);
            right.setPower(r);

            telemetry.addData("Left", l);
            telemetry.addData("Right", r);
            telemetry.update();
            Thread.sleep(10);
        }
        telemetry.addData("Lineup", "Complete");
        telemetry.update();
        left.setPower(0);
        right.setPower(0);

        robot.intakeExtController.hold(robot.ext_max);
        Thread.sleep(100);
        robot.intakePivot.setPosition(robot.pivot_down);
        robot.intake.setPower(-0.85);
        while (robot.intakeExtController.getCurrentPosition() < robot.ext_max - 120) Thread.sleep(5);
        robot.forward(5, 0.3);
        Thread.sleep(500);
        robot.reverse(5, 0.3);
        Thread.sleep(250);
        robot.dunk.setPosition(robot.dunk_up);
        robot.intakePivot.setPosition(robot.pivot_up);
        Thread.sleep(250);
        robot.intakeExtController.hold(0);
        robot.intake.setPower(0);
        Thread.sleep(750);
        robot.dunk.setPosition(robot.dunk_min);

    }
}
