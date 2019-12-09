package org.firstinspires.ftc.teamcode.common.sensors;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.util.Utils;

public class MotorCurrentSensor
{
    private LynxModule revHub;
    private LynxDcMotorController motorController;
    private int port;
    
    private static final LynxGetADCCommand.Channel[] portCommands =
            {
                    LynxGetADCCommand.Channel.MOTOR0_CURRENT,
                    LynxGetADCCommand.Channel.MOTOR1_CURRENT,
                    LynxGetADCCommand.Channel.MOTOR2_CURRENT,
                    LynxGetADCCommand.Channel.MOTOR3_CURRENT
            };
    
    public MotorCurrentSensor(DcMotor motor, LynxModule revHub)
    {
        try
        {
            motorController = (LynxDcMotorController)motor.getController();
            this.revHub = revHub;
            port = motor.getPortNumber();
        }
        catch (ClassCastException e)
        {
            throw new IllegalStateException("Only supports REV motor controllers!");
        }
    }
    
    public int getCurrentDraw()
    {
        try
        {
            LynxGetADCCommand.Channel channel = portCommands[port];
            LynxGetADCResponse resp =
                    new LynxGetADCCommand(revHub, channel, LynxGetADCCommand.Mode.ENGINEERING).sendReceive();
            return resp.getValue();
        }
        catch (InterruptedException | RuntimeException | LynxNackException e)
        {
            return 0;
        }
    }
}
