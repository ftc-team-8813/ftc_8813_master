package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.Persistent;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.sensors.Switch;

/**
 * Robot -- a container for all of the robot hardware interfaces
 */
public class Robot
{
    // Motors
    public final DcMotor leftFront, leftRear;
    public final DcMotor rightFront, rightRear;
    public final DcMotor dunkLift, pullUp;
    public final DcMotor intake;
    public final DcMotor intakePivot;

    // PID controllers
    public final MotorController pivot;

    // Servos
    public final Servo dunk;
    public final Servo hook;
    public final Servo mark;

    // Sensors
    public final IMU imu;
    public final Switch pivotLimit;
    public final Switch liftLimitDown;
    public final Switch liftLimitUp;
    public final Switch pullupLimit;

    // Other
    public final Config config;
    public static final double HOOK_CLOSED = 0.50;
    public static final double HOOK_OPEN = 0;

    public static final double dunk_min = 0.53;
    public static final double dunk_up = 0.75;
    public static final double dunk_dunk = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Initialization and Lifecycle                                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////

    // This is a 'singleton' class. At any time, there can only be one instance of the class.
    // Initialization and de-initialization are handled by the initialize() and uninitialize()
    // functions.
    // To honor the singleton property of this class, instances of Robot should not be used after
    // uninitialize() is called on them.
    private static Robot instance;

    private Robot(HardwareMap hardwareMap, Config config)
    {
        this.config = config;
        // Motors
        leftFront = hardwareMap.dcMotor.get("left front");
        leftRear = hardwareMap.dcMotor.get("left rear");
        dunkLift = hardwareMap.dcMotor.get("dunk lift");
        pullUp = hardwareMap.dcMotor.get("pull up");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightRear = hardwareMap.dcMotor.get("right rear");

        intake = hardwareMap.dcMotor.get("intake");
        intakePivot = hardwareMap.dcMotor.get("intake pivot");

        // Motor controllers
        pivot = new MotorController(intakePivot, config);

        // Servos
        dunk = hardwareMap.servo.get("dunk");
        hook = hardwareMap.servo.get("hook");
        mark = hardwareMap.servo.get("mark");

        hook.setPosition(HOOK_OPEN);

        // Sensors
        if (Persistent.get("imu") != null)
        {
            imu = (IMU)Persistent.get("imu");
        }
        else
        {
            imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
            Persistent.put("imu", imu);
        }
        pivotLimit = new Switch(hardwareMap.digitalChannel.get("pivot limit"));
        liftLimitDown = new Switch(hardwareMap.digitalChannel.get("lower limit"));
        liftLimitUp = new Switch(hardwareMap.digitalChannel.get("upper limit"));
        pullupLimit = new Switch(hardwareMap.digitalChannel.get("pull up limit"));

        // Other

        // Reverse motors as necessary
        if (config.getBoolean("lf_reverse", false)) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("lr_reverse", false)) leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("rf_reverse", false)) rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("rr_reverse", false)) rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("pl_reverse", false)) pullUp.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("dl_reverse", false)) dunkLift.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("in_reverse", false)) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("ip_reverse", false)) intakePivot.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public static Robot initialize(HardwareMap hardwareMap, Config config)
    {
        if (instance != null) instance.uninitialize();
        instance = new Robot(hardwareMap, config);
        return instance;
    }

    public static Robot instance()
    {
        return instance;
    }

    public void uninitialize()
    {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        dunkLift.setPower(0);
        pullUp.setPower(0);
        intake.setPower(0);
        // Stop external threads and close open files (if any) here
        pivot.close();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Functions                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////
    // Intake pivot calibration and control

    public void initPivot() throws InterruptedException
    {
        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakePivot.setPower(-0.4);
        int i = 0;
        while (!pivotLimit.pressed() && i < 5000)
        {
            Thread.sleep(1);
            i++;
        }
        intakePivot.setPower(0);
        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(100);
        pivot.setPower(0.5);
        pivot.hold(15);
    }

    public void initPivotAuto() throws InterruptedException
    {
        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(100);
        pivot.setPower(0.5);
        pivot.hold(15);
    }

    ///////////////////////////////////
    // Autonomous utilities

    public static final double ENC_PER_ROTATION_20  = 537.6; // NeveRest Orbital 20:1 motor
    public static final double ENC_PER_ROTATION_REV = 1120;  // REV Planetary 20:1 motor
    public static final double ENC_PER_INCH = ENC_PER_ROTATION_20 / 25.1327; // For 8in wheels on NeveRest motors
    public static final double ENC_PER_CM = ENC_PER_INCH / 2.54;

    public static final double RADIUS_INCH = 7.75;

    // Driving (distance)
    // NOTE: All driving functions are blocking (i.e. they wait until the robot is done moving)
    // NOTE: All distances are in inches!
    // NOTE: If an interrupt occurs while a driving function is running, the function will exit
    //       but the motors WILL NOT STOP! Make sure to stop motors in your own interrupt handler!

    /**
     * Drive forwards. Like all other driving functions, this function blocks (waits) until the
     * motors are stopped.
     * @param distance How far to drive (in inches)
     * @param power How fast to drive (0 - 1; negative power does NOT drive backwards!)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void forward(double distance, double power) throws InterruptedException
    {
        forwardEnc((int)(distance * ENC_PER_INCH), power);
    }

    /**
     * Drive backwards. Like all other driving functions, this function blocks (waits) until the
     * motors are stopped. Works the same as calling {@link #forward(double, double) forward()} with
     * a negative {@code distance}
     * @param distance How far to drive (in inches)
     * @param power How fast to drive (0 - 1; negative power does NOT drive forwards!)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void reverse(double distance, double power) throws InterruptedException
    {
        forward(-distance, power);
    }

    /**
     * Turns the robot in an arc. Like all other driving functions, this function blocks (waits) until the
     * motors are stopped.
     * <p></p>
     * <p>To do a point turn (one wheel forward, one wheel backwards), set {@code radius} to 0</p>
     * <p>To do a wheel turn (one wheel forward, one wheel stopped), set {@code radius} to {@link Robot#RADIUS_INCH Robot.RADIUS_INCH }</p>
     * <p>To turn counterclockwise, set {@code radius} negative (or {@code angle} negative for point turns)</p>
     * <p><b>Note:</b> Turning in an arc is very inaccurate at the moment, as the faster wheel will drive the slower wheel, increasing
     *   the turn radius and reducing the actual turn angle. Avoid using arc turns when accuracy is needed.</p>
     * @param angle The angle (in radians; 180 degrees = PI radians) to turn
     * @param radius The turn radius (in inches)
     * @param power How fast to turn (-1 - 1; negative power has the same effect as negative angle)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void turn(double angle, double radius, double power) throws InterruptedException
    {
        int distLeft = (int)((radius + RADIUS_INCH) * angle * ENC_PER_INCH * 2);
        int distRight = (int)((radius - RADIUS_INCH) * angle * ENC_PER_INCH * 2);
        turnEnc(distLeft, distRight, power);
    }

    // Driving (encoder ticks)

    /**
     * Drive forward a certain number of encoder ticks. Like all other driving functions, this
     * function blocks (waits) until the motors are stopped.
     * @param distance How far to drive forward (in encoder ticks)
     * @param power How fast to drive (0 - 1; negative power does NOT drive backwards!)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void forwardEnc(int distance, double power) throws InterruptedException
    {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int start = leftRear.getCurrentPosition();

        leftFront.setPower(Math.abs(power) * Math.signum(distance));
        rightFront.setPower(Math.abs(power) * Math.signum(distance));
        leftRear.setPower(Math.abs(power) * Math.signum(distance));
        rightRear.setPower(Math.abs(power) * Math.signum(distance));

        while (Math.abs(leftRear.getCurrentPosition() - start) < Math.abs(distance))
        {
            Thread.sleep(5);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


    }

    // TODO needs testing

    /**
     * Turn a certain number of encoder ticks. More specifically, this function moves the left side
     * and right side different distances simultaneously. Like all other driving functions, this
     * function blocks (waits) until the motors are stopped.
     * @param distLeft How far to drive the left side (in encoder ticks)
     * @param distRight How far to drive the right side (in encoder ticks)
     * @param power How fast to drive (-1 - 1; negative power DOES reverse the motors)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void turnEnc(int distLeft, int distRight, double power) throws InterruptedException
    {
        if (distLeft == 0 && distRight == 0) return; // Avoid division by zero
        // Ensure we're in the correct mode
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double powerLeft, powerRight;

        // Optimize power so that both sides reach their destination at about the same time
        // to avoid strange turn behavior
        if (Math.abs(distLeft) >= Math.abs(distRight))
        {
            powerLeft = power * Math.signum(distLeft);
            powerRight = powerLeft * ((double)distRight / (double)distLeft);
        }
        else
        {
            powerRight = power * Math.signum(distRight);
            powerLeft = powerRight * ((double)distLeft / (double)distRight);
        }

        // Measuring distance from rear motors as they have more traction
        // Motor controller init resets the encoders, so we can start from 0 (yay!)

        MotorController ml = new MotorController(leftRear, config);
        MotorController mr = new MotorController(rightRear, config);

        // Turn off front wheel braking to improve performance since we're just dragging them
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFront.setPower(0);
        ml.setPower(Math.abs(powerLeft));

        rightFront.setPower(0);
        mr.setPower(Math.abs(powerRight));

        ml.startRunToPosition(distLeft);
        mr.startRunToPosition(distRight);

        boolean leftBusy = true, rightBusy = true;

        while (leftBusy || rightBusy)
        {
            try
            {
                if (!ml.isHolding())
                {
                    leftBusy = false;
                    ml.hold(ml.getCurrentPosition());
                }
                if (!mr.isHolding())
                {
                    rightBusy = false;
                    mr.hold(mr.getCurrentPosition());
                }
                Thread.sleep(5);
            }
            catch (InterruptedException e)
            {
                leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                ml.close();
                mr.close();
                throw e;
            }
        }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ml.close();
        mr.close();
    }


}
