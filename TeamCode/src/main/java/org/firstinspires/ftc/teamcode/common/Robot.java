package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Robot -- a container for all of the robot hardware interfaces
 */
public class Robot
{
    // Motors
    public final DcMotor leftFront, leftRear;
    public final DcMotor rightFront, rightRear;
    public final DcMotor leftDunk, rightDunk;
    public final DcMotor intake;

    // Servos
    public final Servo dunk;
    public final Servo intakeFlipper;

    // Sensors

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Initialization and Lifecycle                                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////

    // This is a 'singleton' class. At any time, there can only be one instance of the class.
    // Initialization and de-initialization are handled by the initialize() and uninitialize()
    // functions.
    // To honor the singleton property of this class, instances of Robot should not be used after
    // uninitialize() is called on them.
    private static Robot instance;

    private Robot(HardwareMap hardwareMap)
    {
        // Motors
        leftFront  = hardwareMap.dcMotor.get("left front");
        leftRear   = hardwareMap.dcMotor.get("left rear");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightRear  = hardwareMap.dcMotor.get("right rear");
        leftDunk   = hardwareMap.dcMotor.get("left dunk");
        rightDunk  = hardwareMap.dcMotor.get("right dunk");
        intake     = hardwareMap.dcMotor.get("intake");

        // Servos
        dunk          = hardwareMap.servo.get("dunk");
        intakeFlipper = hardwareMap.servo.get("intake flipper");

        // Sensors

        // Other

        // TODO -- Reverse motors as necessary

    }

    public static Robot initialize(HardwareMap hardwareMap)
    {
        if (instance != null) instance.uninitialize();
        instance = new Robot(hardwareMap);
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
        leftDunk.setPower(0);
        rightDunk.setPower(0);
        intake.setPower(0);
        // Stop external threads and close open files (if any) here

    }

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Functions                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////
    // Autonomous utilities

    public static final double ENC_PER_ROTATION_20  = 537.6; // NeveRest Orbital 20:1 motor
    public static final double ENC_PER_ROTATION_REV = 1120;  // REV Planetary 20:1 motor
    public static final double ENC_PER_INCH = ENC_PER_ROTATION_20 / 25.1327; // For 8in wheels on NeveRest motors
    public static final double ENC_PER_CM = ENC_PER_INCH / 2.54;

    public static final double RADIUS_INCH = 15; // TODO this is a placeholder value that needs to be measured

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
     * @param angle The angle (in radians; 180 degrees = PI radians) to turn
     * @param radius The turn radius (in inches)
     * @param power How fast to turn (-1 - 1; negative power has the same effect as negative angle)
     * @throws InterruptedException If the thread is interrupted (i.e. the user pressed STOP on the
     *                              driver station or the autonomous 30 seconds timed out)
     */
    public void turn(double angle, double radius, double power) throws InterruptedException
    {
        int distLeft = (int)((radius + RADIUS_INCH) * angle);
        int distRight = (int)((radius - RADIUS_INCH) * angle);
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
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + distance);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + distance);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + distance);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + distance);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        while (leftFront.isBusy() || rightFront.isBusy() || leftRear.isBusy() || rightRear.isBusy())
        {
            Thread.sleep(5);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        int startLeft = leftRear.getCurrentPosition();
        int startRight = rightRear.getCurrentPosition();

        leftFront.setPower(powerLeft);
        leftRear.setPower(powerLeft);

        rightFront.setPower(powerRight);
        rightRear.setPower(powerRight);

        boolean leftBusy = true, rightBusy = true;

        while (leftBusy || rightBusy)
        {
            if (Math.abs(leftRear.getCurrentPosition() - startLeft) >= Math.abs(distLeft))
            {
                leftBusy = false;
                leftFront.setPower(0);
                leftRear.setPower(0);
            }
            if (Math.abs(rightRear.getCurrentPosition() - startRight) >= Math.abs(distRight))
            {
                rightBusy = false;
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
            Thread.sleep(5);
        }
    }
}
