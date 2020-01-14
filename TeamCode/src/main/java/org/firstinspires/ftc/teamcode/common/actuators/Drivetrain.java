package org.firstinspires.ftc.teamcode.common.actuators;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.opencv.tracking.TrackerGOTURN;

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
    
    private SpeedController controller;
    private boolean correctAngle;
    
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
        GlobalDataLogger.instance().addChannel("Left Front position", () -> Integer.toString(this.leftFront.getCurrentPosition()));
        GlobalDataLogger.instance().addChannel("Right Front position", () -> Integer.toString(this.rightFront.getCurrentPosition()));
        GlobalDataLogger.instance().addChannel("Left Rear position", () -> Integer.toString(this.leftBack.getCurrentPosition()));
        GlobalDataLogger.instance().addChannel("Right Rear position", () -> Integer.toString(this.rightBack.getCurrentPosition()));
        
        GlobalDataLogger.instance().addChannel("Drivetrain Power", () ->
        {
            double power = Math.abs(this.leftFront.getPower()) + Math.abs(this.rightFront.getPower())
                            + Math.abs(this.leftBack.getPower()) + Math.abs(this.rightBack.getPower());
            return String.format("%.4f", power/4);
        });
        
        GlobalDataLogger.instance().addChannel("Drivetrain State", () -> state);
        controller = new SpeedController(imu, fwdEnc, strafeEnc);
        GlobalThreadPool.instance().start(controller);
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
     * @param turn    How fast to turn counterclockwise (negative for clockwise)
     */
    public void drive(double forward, double right, double turn)
    {
        if (correctAngle)
        {
            if (turn == 0 && controller.getAngleInfluence() != 0) internalEnableCorrection();
            else if (turn != 0) controller.setAngleInfluence(0);
        }
        controller.drive(forward, right, turn);
    }
    
    /**
     * Drive a certain distance in a certain direction
     * @param forward  How fast to drive forward
     * @param right    How fast to strafe
     * @param turn     How fast to turn
     * @param distance How far to move
     * @throws InterruptedException If an interrupt occurs
     */
    public void move(double forward, double right, double turn, double distance) throws InterruptedException
    {
        state = "Move";
        if (turn != 0 && (forward != 0 || right != 0))
        {
            log.e("Arc turns are not supported");
            return;
        }
        if (forward == 0 && right == 0 && (turn == 0 || imu == null))
        {
            return;
        }
        
        controller.move(distance * Math.signum(forward), forward, distance * Math.signum(right), right);
        while (controller.busy)
        {
            Thread.sleep(50);
        }
        
        state = "Idle";
    }
    
    

    public void oldMove(double forward, double right, double turn, int distance) throws InterruptedException
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
                motors[i].getMotor().setPower(powers[i] * Math.signum(distance));
            }
            Thread.sleep(6);
        }

        double angleOrig;
        if (imu != null)
            angleOrig = imu.getHeading();
        else
            angleOrig = 0;
        /*
        if (fwdEnc != null)
        {
            fwdEnc.resetEncoder();
            strafeEnc.resetEncoder();
        }
         */
        PIDMotor encMotor = rightBack;
        int origPos = encMotor.getCurrentPosition();

        // Wait for the motors to finish
        boolean busy = true;
        double prevPowerOff = 0;
        while (busy)
        {
            if (forward != 0 && Math.abs(encMotor.getCurrentPosition() - origPos) >= Math.abs(distance))
                busy = false;
            else if (right != 0 && Math.abs(encMotor.getCurrentPosition() - origPos) >= Math.abs(distance))
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

//            log.d("Encoders: %d %d %d %d",
//                    motors[0].getCurrentPosition(),
//                    motors[1].getCurrentPosition(),
//                    motors[2].getCurrentPosition(),
//                    motors[3].getCurrentPosition());
            Thread.sleep(10);
        }
        motors[0].getMotor().setPower(0);
        motors[1].getMotor().setPower(0);
        motors[2].getMotor().setPower(0);
        motors[3].getMotor().setPower(0);
        angleOffset = 0;
        state = "Idle";
    }

    public AMSEncoder getfrwEnc(){
        return fwdEnc;
    }

    public void stop(){
        drive(0, 0, 0);
    }
    
    
    ////////////////////////////////////
    // Angle Correction
    
    private class SpeedController implements Runnable
    {
        private IMU imu;
        private AMSEncoder fwdEnc, strafeEnc;
        private volatile double targetAngle = 0;
        // private volatile double targetPos;
        private volatile double forward, strafe, turn;
        
        private volatile double fwdTarget, strafeTarget;
        private volatile boolean holdPosition;
        private volatile boolean busy;
        
        private volatile double angleInfluence = 0;
        
        public SpeedController(IMU imu, AMSEncoder fwdEnc, AMSEncoder strafeEnc)
        {
            this.imu = imu;
            this.targetAngle = 0;
            
            this.fwdEnc = fwdEnc;
            this.strafeEnc = strafeEnc;
            
            this.imu.setImmediateStart(true);
            this.imu.initialize();
        }
        
        public synchronized void setAngle(double angle)
        {
            this.targetAngle = angle;
        }
        
        public synchronized void setAngleInfluence(double power)
        {
            this.angleInfluence = Math.abs(power);
        }
        
        public double getAngle()
        {
            return targetAngle;
        }
        
        public double getAngleError()
        {
            return imu.getHeading() - targetAngle;
        }
        
        public double getAngleInfluence()
        {
            return angleInfluence;
        }
        
        public synchronized void drive(double forward, double strafe, double turn)
        {
            this.holdPosition = false;
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
        
        public synchronized void move(double fwdDist, double fwdPower, double strafeDist, double strafePower)
        {
            moveTo(fwdDist + fwdEnc.getAbsoluteAngle(), fwdPower, strafeDist + strafeEnc.getAbsoluteAngle(), strafePower);
        }
        
        public synchronized void moveTo(double fwdPos, double fwdPower, double strafePos, double strafePower)
        {
            fwdTarget = fwdPos;
            strafeTarget = strafePos;
            holdPosition = true;
            busy = true;
            forward = Math.abs(fwdPower);
            strafe = Math.abs(strafePower);
            log.d("moveTo fwd=%.3f strafe=%.3f power=%.3f,%.3f", fwdPos, strafePos, fwdPower, strafePower);
        }
    
        @Override
        public void run()
        {
            double prevFwd = 0, prevStrafe = 0, prevTurn = 0;
            while (true)
            {
                double forward = this.forward;
                double strafe = this.strafe;
                double turn = this.turn;
                
                if (angleInfluence > 0)
                {
                    turn -= getAngleError() / 50 * angleInfluence;
                }
                
                if (holdPosition)
                {
                    double fwdError = fwdEnc.getAbsoluteAngle() - fwdTarget;
                    double strafeError = strafeEnc.getAbsoluteAngle() - strafeTarget;
                    
                    forward *= -Range.clip(fwdError / 120, -1, 1);
                    strafe *= -Range.clip(strafeError / 100, -1, 1);
                    if (Math.abs(forward) < 0.05 && Math.abs(strafe) < 0.05) busy = false;
                }
                
                if (prevFwd != forward || prevStrafe != strafe || prevTurn != turn)
                {
                    prevFwd = forward;
                    prevStrafe = strafe;
                    prevTurn = turn;
    
                    leftFront.getMotor().setPower ( forward + strafe - turn);
                    rightBack.getMotor().setPower ( forward + strafe + turn);
                    rightFront.getMotor().setPower( forward - strafe + turn);
                    leftBack.getMotor().setPower  ( forward - strafe - turn);
                }
    
                try
                {
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    break;
                }
            }
        }
    }
    
    public void enableAngleCorrection()
    {
        internalEnableCorrection();
        correctAngle = true;
    }
    
    private void internalEnableCorrection()
    {
        controller.setAngleInfluence(0.65);
        controller.setAngle(imu.getHeading());
    }
    
    public void disableAngleCorrection()
    {
        correctAngle = false;
        controller.setAngleInfluence(0);
    }
    
    
}
