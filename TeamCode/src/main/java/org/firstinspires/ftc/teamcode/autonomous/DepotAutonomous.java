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
    private static final int LEFT = -1;
    private static final int CENTER = 0;
    private static final int RIGHT = 1;

    private static final String[] sides = {"Left", "Center", "Right"};

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
        // Initialization starts up here so that the camera gets several seconds to warm up

        // Thread.sleep(4000);
        new TaskDrop().runTask();

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.forward(2, 0.2);
        Thread.sleep(500);

        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);

        new TaskDetectGold(detector).runTask();

        new TaskSample(detector).runTask();
        telemetry.clearAll();
        telemetry.update();

        new TaskIntakeMineral().runTask();

        robot.forward(15, 0.2);

        robot.imu.update();
        int side;
        if (robot.imu.getHeading() >= 10) side = LEFT;
        else if (robot.imu.getHeading() <= -10) side = RIGHT;
        else side = CENTER;

        int offset = 0;
        switch (side)
        {
            case LEFT:
                offset = -20;
                break;
            case RIGHT:
                offset = 30;
                break;
            case CENTER:
                offset = 0;
                break;
        }
        telemetry.addData("Side", sides[side+1]);

        for (int i = 0; Math.abs(robot.imu.getHeading() - offset) > 2 && opModeIsActive() && i < 50; )
        {
            if ((robot.imu.getHeading() - offset) > 0)
            {
                left.setPower(0.05);
                right.setPower(-0.05);
            }
            else if ((robot.imu.getHeading() - offset) < 0)
            {
                left.setPower(-0.05);
                right.setPower(0.05);
            }
            else
            {
                left.setPower(0);
                right.setPower(0);
            }
            if (Math.abs(robot.imu.getHeading() - offset) < 2)
            {
                i++;
            }
            Thread.sleep(5);
            telemetry.addData("Side", sides[side+1]);
            robot.imu.update();
        }
        left.setPower(0);
        right.setPower(0);
        robot.forward(20, 0.2);
        Thread.sleep(500);
        robot.mark.setPosition(0.7);

        robot.reverse(15, 0.2);
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
