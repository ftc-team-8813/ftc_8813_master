package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The mecanum drivetrain
 */
public class Drivetrain
{
    private PIDMotor leftFront, rightFront;
    private PIDMotor leftBack,  rightBack;
    
    /**
     * Create a drivetrain. Takes PIDMotors for position control ability
     * @param leftFront  The left front motor
     * @param rightFront The right front motor
     * @param leftBack   The left rear motor
     * @param rightBack  The right rear motor
     */
    public Drivetrain(PIDMotor leftFront, PIDMotor rightFront, PIDMotor leftBack, PIDMotor rightBack)
    {
        this.leftFront  = leftFront;
        this.rightFront = rightFront;
        this.leftBack   = leftBack;
        this.rightBack  = rightBack;
        
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    /**
     * Run the drivetrain at a constant power
     * @param forward How fast to drive forward (negative for backwards)
     * @param right   How fast to strafe to the right (negative for left)
     * @param turn    How fast to turn clockwise (negative for counterclockwise)
     */
    public void drive(double forward, double right, double turn)
    {
        leftFront.getMotor().setPower ( forward + right - turn);
        rightFront.getMotor().setPower( forward - right + turn);
        leftBack.getMotor().setPower  ( forward - right - turn);
        rightBack.getMotor().setPower ( forward + right + turn);
    }
    
    /**
     * Drive a certain distance in a certain direction
     * @param forward  How fast to drive forward
     * @param right    How fast to strafe
     * @param turn     How fast to turn
     * @param distance How far to move
     * @throws InterruptedException If an interrupt occurs
     */
    public void move(double forward, double right, double turn, int distance) throws InterruptedException
    {
        double[] powers = {
                forward + right - turn,
                forward - right + turn,
                forward - right - turn,
                forward + right + turn
        };
        
        PIDMotor[] motors = {leftFront, rightFront, leftBack, rightBack};
        
        // Start the motors
        for (int i = 0; i < 4; i++)
        {
            int dist = (int)(distance * Math.min(Math.abs(powers[i]), 1) * Math.signum(powers[i]));
            int target = dist + motors[i].getCurrentPosition();
            if (powers[i] > 0)
            {
                motors[i].setPower(powers[i]);
                motors[i].startRunToPosition(target);
            }
        }
        
        // Wait for the motors to finish
        boolean busy = true;
        while (busy)
        {
            busy = false;
            for (int i = 0; i < 4; i++)
            {
                if (motors[i].isHolding())
                {
                    busy = true;
                    break;
                }
            }
            Thread.sleep(10);
        }
    }
}
