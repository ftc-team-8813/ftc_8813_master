package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.Charset;

/**
 * PIDTuner - OpMode to adjust PID constants for the MotorController.
 */

@TeleOp(name = "PID Tuner", group = "test")
public class PIDTuner extends OpMode
{
    
    private MotorController controller;
    private ButtonHelper buttons;
    private int changing = 0;
    private long start;

    private DataOutputStream logger;
    
    @Override
    public void init() {
        try {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        controller = new MotorController(hardwareMap.dcMotor.get("intake pivot"), new Config(Config.configFile));
        buttons = new ButtonHelper(gamepad1);
        //controller.setPIDConstants(0, 0, 0);
        controller.setPower(0.5);
        controller.holdStalled(true);
        controller.hold(0);

        File log = new File(Config.storageDir + "pidLog.txt");
        try
        {
            logger = new DataOutputStream(new FileOutputStream(log));
        } catch (FileNotFoundException e)
        {
            logger = null;
        }
        if (logger != null)
        {
            final String[] names = {"kP",      "kI",    "kD",    "target", "position", "error", "integral", "deriv", "output"};
            final int[] colors = {0x7F0000, 0x007F00, 0x00007F,  0xFFFF00,  0x00FF00, 0xFF0000, 0x7F00FF,  0x00FFFF, 0xFFFFFF};
            try
            {
                logger.write("LOGp".getBytes(Charset.forName("UTF-8")));
                logger.writeInt(names.length);
                for (int i = 0; i < names.length; i++)
                {
                    logger.writeInt(colors[i]);
                    logger.write(names[i].getBytes(Charset.forName("UTF-8")));
                    logger.write(0x00); // Null termination
                }
            } catch (IOException e)
            {
                try
                {
                    logger.close();
                }
                catch (IOException e1) {}
                finally
                {
                    logger = null;
                }
            }
        }
    }
    
    @Override
    public void loop() {
        controller.hold(controller.getTargetPosition() - (int)(gamepad1.left_stick_y*50));
        if (buttons.pressing(ButtonHelper.dpad_up)) {
            changing++;
            changing %= 3;
        }
        if (buttons.pressing(ButtonHelper.dpad_down)) {
            changing--;
            if (changing < 0) changing = 2;
        }
        double[] constants = controller.getPIDConstants();
        switch (changing) {
            case 0:
                telemetry.addData("Changing", "Proportional Gain");
                controller.setPIDConstants(constants[0] - gamepad1.right_stick_y / 10000,
                        constants[1], constants[2]);
                break;
            case 1:
                telemetry.addData("Changing", "Integral Gain");
                controller.setPIDConstants(constants[0], constants[1] - gamepad1.right_stick_y / 100000, constants[2]);
                break;
            case 2:
                telemetry.addData("Changing", "Derivative Gain");
                controller.setPIDConstants(constants[0], constants[1], constants[2] -
                        gamepad1.right_stick_y / 10000);
                break;
        }
        if (gamepad1.right_bumper) {
            switch (changing) {
                case 0:
                    controller.setPIDConstants(0, constants[1], constants[2]);
                    break;
                case 1:
                    controller.setPIDConstants(constants[0], 0, constants[2]);
                    break;
                case 2:
                    controller.setPIDConstants(constants[0], constants[1], 0);
                    break;
            }
        }
        telemetry.addData("Target", controller.getTargetPosition());
        telemetry.addData("Position", controller.getCurrentPosition());
        telemetry.addData("Command", controller.getOutput());
        telemetry.addData("kP", constants[0]);
        telemetry.addData("kI", constants[1]);
        telemetry.addData("kD", constants[2]);
        if (gamepad1.left_bumper)
        {
            if (buttons.pressing(ButtonHelper.left_bumper))
            {
                try
                {
                    logger.writeDouble(Double.NaN);
                } catch (IOException e)
                {
                    telemetry.addData("Log error", e.getMessage());
                }
                start = System.nanoTime();
            }
            telemetry.addData("Logging", "");
            log(new double[]{constants[0], constants[1], constants[2],
                    controller.getTargetPosition(), controller.getCurrentPosition(),
                    controller.getInternalController().getError(),
                    controller.getInternalController().getIntegral(),
                    controller.getInternalController().getDerivative(),
                    controller.getOutput()}, start);
        }
    }

    // kP kI kD target position error integral deriv output
    private void log(double[] data, long start)
    {
        try
        {
            if (logger == null) return;
            logger.writeLong(System.nanoTime() - start);
            for (double d : data)
            {
                logger.writeDouble(d);
            }
        }
        catch (IOException e)
        {
            telemetry.addData("Logging error", e.getMessage());
        }
    }
    
    @Override
    public void stop() {
        try
        {
            logger.close();
        } catch (IOException e)
        {
            e.printStackTrace();
        }
        controller.close();
        Logger.close();
    }
}
