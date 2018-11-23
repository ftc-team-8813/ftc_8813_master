package org.firstinspires.ftc.teamcode.autonomous.test.opencv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.util.sensors.vision.ShapeGoldDetector;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="OpenCV test")
public class OpenCVTest extends BaseAutonomous implements CameraStream.OutputModifier
{
    private ShapeGoldDetector detector;
    private volatile int w, h;

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
        stream.addModifier(this);

        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive())
        {
            telemetry.addData("Gold on-screen: ", detector.goldSeen());
            telemetry.addData("Width", w);
            telemetry.addData("Height", h);
            if (detector.getLocation() != null)
            {
                telemetry.addData("Location", detector.getLocation());


                // If our phone is mounted upside down, horizontal = +Y
                if (!detector.goldSeen())
                {
                    left.setPower(0);
                    right.setPower(0);
                    continue;
                }
                // Horizontal error, normalized
                double e = (-detector.getLocation().y / h - 0.5) * 2;
                telemetry.addData("Error", e);

                double l = -0.2;
                double r =  0.2;

                if (e < 0) r -= e * 0.2;
                else       l -= e * 0.2;

                telemetry.addData("Left", l);
                telemetry.addData("Right", r);

                left.setPower(l*2);
                right.setPower(r*2);
            }
            telemetry.update();

        }
    }


    @Override
    public Mat draw(Mat bgr)
    {
        Point location = detector.getLocation();
        if (location != null)
        {
            w = bgr.cols(); // we could make these final, but I don't actually know what the size is!
            h = bgr.rows();
            // Vertical line (blue)
            Imgproc.arrowedLine(bgr, new Point(location.x, 0), new Point(location.x, h-1), new Scalar(0, 0, 255), 1, 8, 0, 0.01);
            // Horizontal line (red)
            Imgproc.arrowedLine(bgr, new Point(0, location.y), new Point(w-1, location.y), new Scalar(255, 0, 0), 1, 8, 0, 0.01);
        }
        return bgr;
    }
}
