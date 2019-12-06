package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;

/**
 * The mecanum drivetrain
 */
public class Drivetrain
{
    public PIDMotor leftFront, rightFront;
    public PIDMotor leftBack,  rightBack;
    
    private Logger log = new Logger("Drivetrain");
    
    private IMU imu;
    
    private AMSEncoder fwdEnc, strafeEnc;
    
    private volatile String state = "Idle";
    private volatile double angleOffset = 0;
    
    private double acceleration;
    
    /**
     * Create a drivetrain. Takes PIDMotors for position control ability
     * @param leftFront  The left front motor
     * @param rightFront The right front motor
     * @param leftBack   The left rear motor
     * @param rightBack  The right rear motor
     */
    public Drivetrain(PIDMotor leftFront, PIDMotor rightFront, PIDMotor leftBack, PIDMotor rightBack, IMU imu, AMSEncoder fwdEnc, AMSEncoder strafeEnc)
    {
        this.leftFront  = leftFront;
        this.rightFront = rightFront;
        this.leftBack   = leftBack;
        this.rightBack  = rightBack;
        
        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = imu;
        
        this.fwdEnc = fwdEnc;
        this.strafeEnc = strafeEnc;
        
        this.leftFront.setDeadband(20);
        this.rightFront.setDeadband(20);
        this.leftBack.setDeadband(20);
        this.rightBack.setDeadband(20);
        GlobalDataLogger.instance().addChannel("Drivetrain State", () -> state);
        GlobalDataLogger.instance().addChannel("Drivetrain Angle Offset", () -> String.format("%.4f", angleOffset));
    }
    
    /**
     * Create a drivetrain. Takes PIDMotors for position control ability
     * @param leftFront  The left front motor
     * @param rightFront The right frontI2cDeviceSynch motor
     * @param leftBack   The left rear motor
     * @param rightBack  The right rear motor
     */
    public Drivetrain(PIDMotor leftFront, PIDMotor rightFront, PIDMotor leftBack, PIDMotor rightBack)
    {
        this(leftFront, rightFront, leftBack, rightBack, null, null, null);
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
     * @param distance How far to oldMove
     * @throws InterruptedException If an interrupt occurs
     */
    public void move(double forward, double right, double turn, int distance) throws InterruptedException
    {
        state = "Move";
        if (turn != 0 && (forward != 0 || right != 0))
        {
            log.e("Arc turns are not supported");
            return;
        }
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
            if (powers[i] != 0)
            {
                motors[i].getMotor().setPower(powers[i]);
            }
            Thread.sleep(6);
        }
        
        double angleOrig;
        if (imu != null)
            angleOrig = imu.getHeading();
        else
            angleOrig = 0;
        
        if (fwdEnc != null)
        {
            fwdEnc.resetEncoder();
            strafeEnc.resetEncoder();
        }
        
        // Wait for the motors to finish
        boolean busy = true;
        double prevPowerOff = 0;
        while (busy)
        {
            if (forward != 0 && fwdEnc != null && Math.abs(fwdEnc.getAbsoluteAngle()) >= Math.abs(distance))
                busy = false;
            else if (right != 0 && strafeEnc != null && Math.abs(strafeEnc.getAbsoluteAngle()) >= Math.abs(distance))
                busy = false;
            else if (turn != 0 && imu != null && Math.abs(imu.getHeading() - angleOrig) >= Math.abs(distance))
                busy = false;
            else
                busy = true;
            
            
            // TODO TEST EXPERIMENTAL CODE
            // Adjust speed to correct for any rotation
            /*
            if (imu != null && turn == 0)
            {
                double angleError = imu.getHeading() - angleOrig;
                double powerOffset = angleError * 0.005 * Math.signum(distance);
                
                if (powerOffset != prevPowerOff)
                {
                    prevPowerOff = powerOffset;
                    log.d("Angle offset: %.2f (add %.3f power)", angleError, powerOffset);
                    motors[0].getMotor().setPower(Math.abs(powers[0]) + powerOffset);
                    motors[1].getMotor().setPower(Math.abs(powers[1]) - powerOffset);
                    motors[2].getMotor().setPower(Math.abs(powers[2]) + powerOffset);
                    motors[3].getMotor().setPower(Math.abs(powers[3]) - powerOffset);
                }
                
                angleOffset = angleError; // For logging
            }
             */
            // ----------------------
            
            log.d("Encoders: %d %d %d %d",
                    motors[0].getCurrentPosition(),
                    motors[1].getCurrentPosition(),
                    motors[2].getCurrentPosition(),
                    motors[3].getCurrentPosition());
            Thread.sleep(10);
        }
        motors[0].getMotor().setPower(0);
        motors[1].getMotor().setPower(0);
        motors[2].getMotor().setPower(0);
        motors[3].getMotor().setPower(0);
        angleOffset = 0;
        state = "Idle";
    }

    public void oldMove(double forward, double right, double turn, int distance) throws InterruptedException
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
            int sign = (motors[i].getMotor().getDirection() == DcMotorSimple.Direction.FORWARD) ? 1 : -1;
            int dist = distance * (int)Math.signum(powers[i]); // (int)(distance * Math.min(Math.abs(powers[i]), 1) * Math.signum(powers[i]));
            int target = (dist + motors[i].getCurrentPosition()); //* sign;
            log.d("Motor %d: target %d (distance %d from %d)", i, target, dist, motors[i].getCurrentPosition());
            if (powers[i] != 0)
            {
                motors[i].setPower(Math.abs(powers[i]));
                motors[i].hold(target);
            }
        }

        double angleOrig;
        if (imu != null)
            angleOrig = imu.getHeading();
        else
            angleOrig = 0;

        // Wait for the motors to finish
        boolean busy = true;
        double prevPowerOff = 0;
        while (busy)
        {
            busy = false;
            for (int i = 0; i < 4; i++)
            {
                if (motors[i].isBusy())
                {
                    busy = true;
                    break;
                }
            }

            // TODO TEST EXPERIMENTAL CODE
            // Adjust speed to correct for any rotation
            if (imu != null && turn == 0)
            {
                double angleError = imu.getHeading() - angleOrig;
                double powerOffset = angleError * 0.02 * Math.signum(distance);

                if (powerOffset != prevPowerOff)
                {
                    prevPowerOff = powerOffset;
                    log.d("Angle offset: %.2f (add %.3f power)", angleError, powerOffset);
                    motors[0].setPower(Math.abs(powers[0]) + powerOffset);
                    motors[1].setPower(Math.abs(powers[1]) - powerOffset);
                    motors[2].setPower(Math.abs(powers[2]) + powerOffset);
                    motors[3].setPower(Math.abs(powers[3]) - powerOffset);
                }

            }
            // ----------------------



            log.d("Encoders: %d %d %d %d",
                    motors[0].getCurrentPosition(),
                    motors[1].getCurrentPosition(),
                    motors[2].getCurrentPosition(),
                    motors[3].getCurrentPosition());
            Thread.sleep(10);
        }
        motors[0].stopHolding();
        motors[1].stopHolding();
        motors[2].stopHolding();
        motors[3].stopHolding();
    }

    public AMSEncoder getfrwEnc(){
        return fwdEnc;
    }

    public void stop(){
        leftFront.getMotor().setPower(0);
        leftBack.getMotor().setPower(0);
        rightFront.getMotor().setPower(0);
        rightBack.getMotor().setPower(0);
    }
}
