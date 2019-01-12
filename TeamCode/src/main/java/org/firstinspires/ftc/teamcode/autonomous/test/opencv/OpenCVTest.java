package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskFindGold;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="OpenCV test")
public class OpenCVTest extends BaseAutonomous
{
    private ShapeGoldDetector detector;
    private static final int w = 640, h = 480;

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
        CameraStream stream = getCameraStream();
        detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);

        DcMotor left = hardwareMap.dcMotor.get("left rear");
        DcMotor right = hardwareMap.dcMotor.get("right rear");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor lf = hardwareMap.dcMotor.get("left front");
        DcMotor rf = hardwareMap.dcMotor.get("right front");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        new TaskFindGold(left, right, detector, true).runTask();
    }
}
