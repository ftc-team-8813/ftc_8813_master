package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskFindGold;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.common.util.sensors.vision.ShapeGoldDetector;

@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        DcMotor left = hardwareMap.dcMotor.get("left rear");
        DcMotor right = hardwareMap.dcMotor.get("right rear");
        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

        DcMotor leftF = hardwareMap.dcMotor.get("left front");
        DcMotor rightF = hardwareMap.dcMotor.get("right front");
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize camera
        CameraStream stream = getCameraStream();
        ShapeGoldDetector detector = new ShapeGoldDetector();
        stream.addModifier(detector);
        stream.addListener(detector);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(-0.1);
        right.setPower(0.1);
        while (!detector.goldSeen() && opModeIsActive()) Thread.sleep(5);
        left.setPower(0);
        right.setPower(0);

        right.setDirection(DcMotorSimple.Direction.FORWARD);
        new TaskFindGold(left, right, detector).runTask();
        telemetry.clearAll();
        telemetry.update();
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setPower(0.5);
        right.setPower(0.5);
        Thread.sleep(1000);
        left.setPower(0);
        right.setPower(0);
    }
}
