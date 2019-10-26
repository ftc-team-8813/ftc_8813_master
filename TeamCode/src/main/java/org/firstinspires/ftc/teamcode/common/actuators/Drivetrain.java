package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drivetrain
{
    private DcMotor leftFront, rightFront;
    private DcMotor leftBack,  rightBack;
    
    public Drivetrain(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack)
    {
        this.leftFront  = leftFront;
        this.rightFront = rightFront;
        this.leftBack   = leftBack;
        this.rightBack  = rightBack;
        
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void drive(double forward, double right, double turn)
    {
        leftFront.setPower ( forward + right - turn);
        rightFront.setPower( forward - right + turn);
        leftBack.setPower  ( forward - right - turn);
        rightBack.setPower ( forward + right + turn);
    }
}
