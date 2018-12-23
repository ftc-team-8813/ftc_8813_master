package org.firstinspires.ftc.teamcode.common.util.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * ACS714 +/- 5A current sensor wrapper
 */
public class CurrentSensor
{
    //Sensor input
    private AnalogInput input;
    
    /**
     * Create a current sensor
     *
     * @param input The input port to read input voltages
     */
    public CurrentSensor(AnalogInput input)
    {
        this.input = input;
    }
    
    /**
     * Get the voltage of the analog input
     *
     * @return the voltage of the analog input, in volts
     */
    public double getInputVoltage()
    {
        return input.getVoltage();
    }
    
    /**
     * Get the current flow through the sensor. The description says that:
     * <blockquote>
     * When Vcc is 5 V, this output voltage is centered at 2.5 V and changes by 185 mV per amp
     * of input current, with positive current increasing the output voltage and negative current
     * decreasing the output voltage.
     * </blockquote>
     *
     * @return the current flow through the sensor, in amperes
     */
    public double getCurrent()
    {
        return (getInputVoltage() - 2.5) / 0.185;
    }
}
