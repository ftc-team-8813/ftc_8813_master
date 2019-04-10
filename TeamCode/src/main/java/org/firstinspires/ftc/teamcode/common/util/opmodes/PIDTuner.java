package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.util.Chooser;
import org.firstinspires.ftc.teamcode.common.util.MotorController;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.io.File;
import java.io.IOException;

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

    private Chooser chooser;
    
    @Override
    public void init() {
        try {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        log = new Logger("PID Tuner");
        buttons = new ButtonHelper(gamepad1);


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
    }

    @Override
    public void init_loop()
    {
        if (chooser == null)
        {
            String[] motors = Utils.allDeviceNames(hardwareMap.dcMotor);
            chooser = new Chooser("Choose a motor and then press PLAY", motors, gamepad1, telemetry);
            chooser.setEnterButton(-1);
        }
        chooser.update();
    }

    @Override
    public void start()
    {
        chooser.choose();
        controller = new MotorController.MotorControllerFactory(hardwareMap.dcMotor.get((String)chooser.getSelected())).create();
        controller.setPIDConstants(0, 0, 0);
        controller.setPower(0.5);
        controller.holdStalled(true);
        controller.hold(0);
    }


//    log(new double[]{kP, kI, kD,
//            controller.getTargetPosition(), controller.getCurrentPosition(),
//            controller.getInternalController().getError(),
//            controller.getInternalController().getIntegral(),
//            controller.getInternalController().getDerivative(),
//            controller.getOutput()}, start);
    
    @Override
    public void loop() {
        controller.hold(controller.getTargetPosition() - (int)(gamepad1.left_stick_y*10));
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
        try
        {
            Thread.sleep(5);
        }
        catch (InterruptedException e)
        {
            requestOpModeStop();
        }
    }

    // kP kI kD target position error integral deriv output

    
    @Override
    public void stop()
    {
        log.i("Finished PID tuning for %s", (String)chooser.getSelected());
        log.i("Final PID constants:");
        double[] cons = controller.getPIDConstants();
        log.i("kP: %.6f, kI: %.6f, kD: %.6f", cons[0], cons[1], cons[2]);
        dataLogger.close();
        // Allow the interrupt to be interpreted
        controller.close();
        Logger.close();
    }
}
