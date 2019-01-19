package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataLogger;
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

    private DataLogger dataLogger;
    private Logger log;
    
    @Override
    public void init() {
        try {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        log = new Logger("PID Tuner");
        controller = new MotorController(hardwareMap.dcMotor.get("intake pivot"), new Config(Config.configFile));
        buttons = new ButtonHelper(gamepad1);
        controller.setPIDConstants(0, 0, 0);
        controller.setPower(0.5);
        controller.holdStalled(true);
        controller.hold(0);

        try
        {
            dataLogger = new DataLogger(new File(Config.storageDir + "pidLog.dat"),
                    new DataLogger.Channel("kP",       0xAA0000),
                    new DataLogger.Channel("kI",       0x00AA00),
                    new DataLogger.Channel("kD",       0x0000AA),
                    new DataLogger.Channel("target",   0xFFFF00),
                    new DataLogger.Channel("position", 0x00FF00),
                    new DataLogger.Channel("error",    0xFF0000),
                    new DataLogger.Channel("integral", 0x7F00FF),
                    new DataLogger.Channel("deriv",    0x00FFFF),
                    new DataLogger.Channel("output",   0xFFFFFF));

        } catch (IOException e)
        {
            throw new RuntimeException(e);
        }
    }
//    log(new double[]{kP, kI, kD,
//            controller.getTargetPosition(), controller.getCurrentPosition(),
//            controller.getInternalController().getError(),
//            controller.getInternalController().getIntegral(),
//            controller.getInternalController().getDerivative(),
//            controller.getOutput()}, start);
    
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
            telemetry.addData("Logging", "");
        }
        else
        {
            dataLogger.stopLogging();
        }
        if (buttons.pressing(ButtonHelper.left_bumper))
        {
            dataLogger.startLogging(new DataLogger.LogCallback()
            {
                @Override
                public void putData(double[] array)
                {
                    double[] constants = controller.getPIDConstants();
                    array[0] = constants[0];
                    array[1] = constants[1];
                    array[2] = constants[2];
                    array[3] = controller.getTargetPosition();
                    array[4] = controller.getCurrentPosition();
                    array[5] = controller.getInternalController().getError();
                    array[6] = controller.getInternalController().getIntegral();
                    array[7] = controller.getInternalController().getDerivative();
                    array[8] = controller.getOutput();
                }
            });
        }
    }

    // kP kI kD target position error integral deriv output

    
    @Override
    public void stop() {

        try
        {
            dataLogger.close();
        } catch (IOException e)
        {
            log.e(e);
        }
        // Allow the interrupt to be interpreted
        controller.close();
        Logger.close();
    }
}
