package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskFindGold;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.opencv.CameraStream;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.sensors.vision.GoldDetector;
import org.firstinspires.ftc.teamcode.util.sensors.vision.ShapeGoldDetector;

@Autonomous(name="Autonomous")
public class MainAutonomous extends BaseAutonomous
{
    @Override
    public void run() throws InterruptedException
    {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotor lifter = hardwareMap.dcMotor.get("lifter");

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
        Thread.sleep(100013502);
        left.setPower(0);
        right.setPower(0);
    }
}
