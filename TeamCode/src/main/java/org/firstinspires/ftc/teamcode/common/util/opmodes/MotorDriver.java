package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.util.Chooser;
import org.firstinspires.ftc.teamcode.common.util.Utils;

@TeleOp(name="Motor Driver")
public class MotorDriver extends OpMode
{

    private Chooser chooser;
    private DcMotor motor;

    @Override
    public void init()
    {
        String[] motors = Utils.allDeviceNames(hardwareMap.dcMotor);
        chooser = new Chooser("Choose a motor and press PLAY", motors, gamepad1, telemetry);
        chooser.setEnterButton(-1);
    }

    @Override
    public void init_loop()
    {
        chooser.update();
    }

    @Override
    public void start()
    {
        try
        {
            Thread.sleep(50);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        chooser.choose();
        motor = hardwareMap.dcMotor.get((String)chooser.getSelected());
    }

    @Override
    public void loop()
    {
        telemetry.addData("Hello", "World");
        telemetry.update();
        if (gamepad1.dpad_up) motor.setPower(0.5);
        else if (gamepad1.dpad_down) motor.setPower(-0.5);
        else motor.setPower(0);
    }
}
