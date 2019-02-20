package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskSample;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Vlogger;
import org.firstinspires.ftc.teamcode.common.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;

@Autonomous(name="OpenCV test")
public class OpenCVTest extends BaseAutonomous implements CameraStream.OutputModifier
{
    private ShapeGoldDetector detector;
    private static final int w = 640, h = 480;
    private Vlogger vlogger;

    /*
    Vision Coordinate System:

    (phone upright)
                ^ -X   *
                |       (0,0)
                |
         <------+------>
         +Y     |      -Y
                |
                v +X
     */
    @Override
    public void run() throws InterruptedException
    {
        vlogger = new Vlogger("opencv_test.avi", 480, 640, 10.0);
        Robot robot = Robot.instance();
        CameraStream stream = getCameraStream();
        detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);
        stream.addModifier(this);

        DcMotor left = robot.leftFront;
        DcMotor right = robot.rightFront;
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor lf = robot.leftFront;
        DcMotor rf = robot.rightFront;
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        new TaskSample(detector, true).runTask();
    }

    @Override
    public void finish()
    {
        if (vlogger != null) vlogger.close();
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
