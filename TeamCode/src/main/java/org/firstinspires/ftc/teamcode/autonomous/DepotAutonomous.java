package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDetectGold;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskDrop;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskIntakeMineral;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.WebcamStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Profiler;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.common.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.io.File;

@Autonomous(name="Depot Autonomous")
public class DepotAutonomous extends BaseAutonomous implements CameraStream.OutputModifier
{
    private Vlogger vlogger;
    private static final int LEFT = -1;
    private static final int CENTER = 0;
    private static final int RIGHT = 1;

    private int side;

    private static final String[] sides = {"Left", "Center", "Right"};

    private Profiler profiler = new Profiler();
    private Logger log = new Logger("Depot Autonomous");

    public static final boolean DROP = false;

    @Override
    public void initialize() throws InterruptedException
    {
        Robot robot = Robot.instance();
        robot.hook.setPosition(Robot.HOOK_CLOSED);
        robot.imu.initialize(telemetry);
        robot.imu.start();

        robot.initPivotAuto();
        CameraStream stream = getCameraStream();
        vlogger = new Vlogger(getVlogName(),
                (int)stream.getSize().width, (int)stream.getSize().height, 10.0);
        robot.mark.setPosition(0.2);
    }

    private String getVlogName()
    {
        int i;
        for (i = 0; new File(Config.storageDir + "autonomous_capture" + i + ".avi").exists(); i++);
        return "autonomous_capture" + i + ".avi";
    }

    @Override
    public void run() throws InterruptedException
    {
        profiler.start("run");
        Robot robot = Robot.instance();
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        profiler.start("init camera");
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        profiler.end();

        profiler.start("drop");
        if (DROP) new TaskDrop().runTask();
        else Thread.sleep(4000);
        robot.imu.resetHeading();
        profiler.end();

        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);

        profiler.start("detect");
        new TaskDetectGold(detector, profiler).runTask();
        profiler.end();

        profiler.start("sample");
        new TaskSample(detector).runTask();
        profiler.end();
        telemetry.clearAll();
        telemetry.update();

        profiler.start("intake");
        new TaskIntakeMineral(profiler).runTask();
        profiler.end();

        profiler.start("drop marker");
        profiler.start("forward");
        robot.forward(35, 0.3);
        profiler.end();

        robot.imu.update();
        if (robot.imu.getHeading() >= 25) side = LEFT;
        else if (robot.imu.getHeading() <= -30) side = RIGHT;
        else side = CENTER;

        int offset = 0;
        switch (side)
        {
            case LEFT:
                offset = -15;
                break;
            case RIGHT:
                offset = 30;
                break;
            case CENTER:
                offset = 0;
                break;
        }
        telemetry.addData("Side", sides[side+1]).setRetained(true);
        profiler.start("turn");
        turnTo(offset);
        profiler.end();

        profiler.start("forward2");
        robot.forward(35, 0.4);
        Thread.sleep(200);
        profiler.end();
        robot.mark.setPosition(0.7);

        profiler.end(); // drop marker

        profiler.start("park");

        profiler.start("turn 2");
        turnTo(52);
        profiler.end();

        profiler.start("back up 2");
        robot.reverse(125, 0.6);
        profiler.end();
        profiler.end(); // park
        profiler.end(); // run
    }

    private void turnTo(int offset) throws InterruptedException
    {
        Robot robot = Robot.instance();
        DcMotor left = robot.leftFront;
        DcMotor right = robot.rightFront;
        double speed = 0.22;
        for (int i = 0; Math.abs(robot.imu.getHeading() - offset) > 2 && opModeIsActive() && i < 50; )
        {
            if ((robot.imu.getHeading() - offset) > 0)
            {
                left.setPower(speed);
                right.setPower(-speed);
            }
            else if ((robot.imu.getHeading() - offset) < 0)
            {
                left.setPower(-speed);
                right.setPower(speed);
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
            else
            {
                i = 0;
            }
            Thread.sleep(5);
            robot.imu.update();
        }
        left.setPower(0);
        right.setPower(0);
    }

    @Override
    public void finish() throws InterruptedException
    {
        vlogger.close();
        log.d("Depot Autonomous finished -- mineral=%s, drop=%s", sides[side+1], Boolean.toString(DROP));
        profiler.finish();
    }

    @Override
    public Mat draw(Mat bgr)
    {
        Mat m2 = new Mat();
        if (getCameraStream() instanceof WebcamStream)
            bgr.copyTo(m2);
        else
            Core.rotate(bgr, m2, Core.ROTATE_90_COUNTERCLOCKWISE);
        vlogger.put(m2);
        m2.release();
        return bgr;
    }

}
