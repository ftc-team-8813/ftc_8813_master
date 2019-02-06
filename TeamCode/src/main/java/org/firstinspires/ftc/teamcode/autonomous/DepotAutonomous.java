package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectGold;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskIntakeMineral;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.videoio.VideoWriter;

import java.io.File;

@Autonomous(name="Depot Autonomous")
public class DepotAutonomous extends BaseAutonomous implements CameraStream.OutputModifier
{
    private Vlogger vlogger;

    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.imu.initialize(telemetry);
        robot.imu.start();

        robot.initPivot();
        vlogger = new Vlogger("autonomous_capture.avi", 480, 640, 10.0);
        robot.mark.setPosition(0);
    }

    @Override
    public void run() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);
        // Initialization starts up here so that the camera gets several seconds to warm up

        Thread.sleep(4000);
        // new TaskDrop().runTask();

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.forward(5, 0.25);

        new TaskDetectGold(detector).runTask();

        new TaskSample(detector).runTask();
        telemetry.clearAll();
        telemetry.update();

        new TaskIntakeMineral().runTask();

        robot.forward(15, 0.2);

        while (Math.abs(robot.imu.getHeading()) > 2 && opModeIsActive())
        {
            if (robot.imu.getHeading() > 0)
            {
                left.setPower(0.05);
                right.setPower(-0.05);
            }
            else
            {
                left.setPower(-0.05);
                right.setPower(0.05);
            }
            Thread.sleep(5);
            robot.imu.update();
        }
        left.setPower(0);
        right.setPower(0);
        robot.forward(15, 0.2);
        Thread.sleep(500);
        robot.mark.setPosition(0.7);
    }

    @Override
    public void finish() throws InterruptedException
    {
        vlogger.close();
    }

    @Override
    public Mat draw(Mat bgr)
    {
        Mat m2 = new Mat();
        Core.rotate(bgr, m2, Core.ROTATE_90_COUNTERCLOCKWISE);
        vlogger.put(m2);
        m2.release();
        return bgr;
    }

}
