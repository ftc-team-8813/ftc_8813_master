package org.firstinspires.ftc.teamcode.autonomous.tasks;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Joseph Murphy on 10/4/2017.
 */

public class TaskMoveArm implements Task{
    DcMotor base;
    Servo arm1;
    Servo arm2;
    Servo claw;
    double x;
    double y;
    double z;
    double arm_length = 420.69;
    public TaskMoveArm(DcMotor base, double x, double y, double z) {this.base = base; this.x = x; this.y = y; this.z = z;}
    @Override
    public void runTask() throws InterruptedException {
        /*
            Base Rotation:
            Finds the cotangent of the x and y points
            and rotates the motor towards the point.
        */
        Double tangent = Math.toDegrees(Math.atan(y/x));
        int rotation = tangent.intValue();
        if(x >= 0 && y >= 0){
            rotation = 90 - rotation;
        }else if(x <= 0 && y >= 0){
            rotation = 360 + rotation;
        }else if(x >= 0 && y <= 0){
            rotation = 180 - rotation;
        }else if(x <= 0 && y <= 0){
            rotation = 180 + rotation;
        }
        base.setTargetPosition(rotation);

        /*
            Extension of Arm:
            Work in Progress
        */
        double XYLength = Math.sqrt((x*x) + (y*y));
        double XYZLength = Math.sqrt((XYLength*XYLength) + (z*z));
        if(XYLength > arm_length){
            //Extend base drawer slides by XYLength - arm_length
        }
        
    }
}