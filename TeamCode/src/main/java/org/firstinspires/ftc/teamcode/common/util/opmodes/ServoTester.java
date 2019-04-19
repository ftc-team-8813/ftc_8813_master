package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Servo Test")
@Disabled
public class ServoTester extends LinearOpMode
{
    private List<ServoController> servoControllers;

    @Override
    public void runOpMode() throws InterruptedException
    {
        servoControllers = new ArrayList<>();
        String[] names = Utils.allDeviceNames(hardwareMap.servoController);
        for (String s : names)
        {
            servoControllers.add(hardwareMap.servoController.get(s));
        }
        telemetry.addData("Servo controllers", names.length);
        telemetry.update();

        waitForStart();
        telemetry.addData("Test 1", "Set all to 0");
        telemetry.update();
        Thread.sleep(1000);
        int i = 0;
        for (ServoController cnt : servoControllers)
        {
            for (int port = 0; port < 6; port++)
            {
                cnt.setServoPosition(port, 0);
                telemetry.addData("Controller", i);
                telemetry.addData("Port", port);
                telemetry.update();
                Thread.sleep(3000);
            }
            i++;
        }

        while (opModeIsActive())
        {
            Thread.sleep(10);
        }
    }
}
