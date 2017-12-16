package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.DcMotorUtil;

import static java.lang.Thread.sleep;

/**
 * Created by Joseph Murphy on 12/1/2017.
 */

public class TaskScoreJewel implements Task{
    DcMotor base;
    Servo colorArm;
    boolean isBlue;
    //I2cDevice colorSensor;
    //I2cDeviceSynch colorSensorReader;
    ColorSensor colorSensor;
    //byte[] color;
    public TaskScoreJewel(int quadrant){
        HardwareMap hardwareMap = BaseAutonomous.instance().hardwareMap;
        base = hardwareMap.dcMotor.get("base");
        //colorSensor = hardwareMap.i2cDevice.get("color_sensor");
        //colorSensorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        colorArm = hardwareMap.servo.get("colorArm");
        if(quadrant == 2 || quadrant == 4){
            isBlue = false;
        }else{
            isBlue = true;
        }
    }
    @Override
    public void runTask() throws InterruptedException {
        boolean isRed = false;
        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //if(isBlue){move(180, .25);} //4200
        //else{move(180, .25);}
        colorArm.setPosition(.75);
        /*colorSensorReader.engage();
        color = colorSensorReader.read(0x04, 1);
        if(color[0] < 4){
            isRed = false;
        }else{
            isRed = true;
        }*/
        if(colorSensor.red() > colorSensor.blue()){
            isRed = true;
        }else if (colorSensor.blue() > colorSensor.red()){
            isRed = false;
        }
        if(isBlue) {
            if(isRed){
                move(90, .5);
            }else{
                move(270, .5);
            }
        }else{
            if(isRed){
                move(90, .5);
            }else{
                move(270, .5);
            }
        }
    }
    public void move(double degrees, double power){
        base.setTargetPosition(DcMotorUtil.degreesToEncoders(degrees, 30));
        base.setPower(power);
    }
}