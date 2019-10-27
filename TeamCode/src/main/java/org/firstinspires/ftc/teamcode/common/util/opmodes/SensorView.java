package org.firstinspires.ftc.teamcode.common.util.opmodes;

import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Utils;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Consumer;

@TeleOp(name="Sensor View")
public class SensorView extends BaseTeleOp
{
    private Logger log;
    private List<DcMotorController> motorControllers;
    private List<DigitalChannelController> digitalControllers;
    private List<AnalogInputController> analogControllers;
    
    @Override
    public void init()
    {
        super.init();
        motorControllers = new ArrayList<>();
        digitalControllers = new ArrayList<>();
        analogControllers = new ArrayList<>();
        
        log = new Logger("SensorView");
        for (DcMotorController module : hardwareMap.getAll(LynxDcMotorController.class))
        {
            log.d("%s: at %s", module.getDeviceName(), module.getConnectionInfo());
            motorControllers.add(module);
        }
        for (DigitalChannelController module : hardwareMap.getAll(LynxDigitalChannelController.class))
        {
            log.d("%s: at %s", module.getDeviceName(), module.getConnectionInfo());
            digitalControllers.add(module);
            for (int i = 0; i < 8; i++)
            {
                module.setDigitalChannelMode(i, DigitalChannel.Mode.INPUT);
            }
        }
        for (AnalogInputController module : hardwareMap.getAll(LynxAnalogInputController.class))
        {
            log.d("%s: at %s", module.getDeviceName(), module.getConnectionInfo());
            analogControllers.add(module);
        }
    }
    
    @Override
    public void doLoop()
    {
        telemetry.clearAll();
        for (DcMotorController module : motorControllers)
        {
            telemetry.addLine(module.getDeviceName() + ": " + module.getConnectionInfo());
            for (int j = 0; j < 4; j++)
            {
                telemetry.addLine("Position: " + module.getMotorCurrentPosition(j));
            }
            telemetry.addLine();
        }
        
        for (DigitalChannelController module : digitalControllers)
        {
            telemetry.addLine();
            telemetry.addLine(module.getDeviceName() + ": " + module.getConnectionInfo());
            StringBuilder b = new StringBuilder();
            for (int j = 0; j < 8; j++)
            {
                b.append(module.getDigitalChannelState(j) ? 1 : 0);
                b.append(" ");
            }
            telemetry.addLine(b.toString());
        }
        
        telemetry.addLine();
        
        for (AnalogInputController module : analogControllers)
        {
            telemetry.addLine();
            telemetry.addLine(module.getDeviceName() + ": " + module.getConnectionInfo());
            for (int j = 0; j < 4; j++)
            {
                telemetry.addLine(String.format(Locale.US, "%.3f", module.getAnalogInputVoltage(j)));
            }
        }
    }
}
