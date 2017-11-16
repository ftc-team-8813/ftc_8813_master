package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joseph Murphy on 11/12/2017.
 */

public class TaskScoreJewel implements Task{

    boolean isBlue;
    Servo servo1;
    Servo servo2;
    I2cDevice colorSensor;
    I2cDeviceSynch colorSensorReader;
    byte[] color;
    public TaskScoreJewel(boolean isBlue, Servo servo1, Servo servo2, I2cDevice colorSensor){
        this.isBlue = isBlue;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.colorSensor = colorSensor;
        colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
    }
    @Override
    public void runTask() throws InterruptedException {
        boolean isRed = true;
        servo1.setPosition(.5);
        colorSensorReader.engage();
        color = colorSensorReader.read(0x04, 1);

        if(color[0] < 4){
            isRed = false;
        }else if(color[0] < 9 && color[0] > 11){
            isRed = true;
        }
        if(isBlue) {
            if (isRed) {
                servo2.setPosition(.25);
            } else {
                servo2.setPosition(.75);
            }
        }
        if(!isBlue){
            if(isRed){
                servo2.setPosition(.75);
            }else{
                servo2.setPosition(.25);
            }
        }
    }
}
