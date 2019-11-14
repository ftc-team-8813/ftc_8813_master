package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.util.Logger;

/**
 * The mecanum drivetrain
 */
public class Drivetrain
{
    public PIDMotor leftFront, rightFront;
    public PIDMotor leftBack,  rightBack;
    
    private Logger log = new Logger("Drivetrain");
    
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
            // int sign = (motors[i].getMotor().getDirection() == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
            int dist = distance * (int)Math.signum(powers[i]); // (int)(distance * Math.min(Math.abs(powers[i]), 1) * Math.signum(powers[i]));
            int target = (dist + motors[i].getCurrentPosition()); //* sign;
            if (powers[i] != 0)
            {
                motors[i].setPower(Math.abs(powers[i]));
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
            log.d("Encoders: %d %d %d %d",
                    motors[0].getCurrentPosition(),
                    motors[1].getCurrentPosition(),
                    motors[2].getCurrentPosition(),
                    motors[3].getCurrentPosition());
            Thread.sleep(10);
        }
    }
}
