package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.MotorController;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.Persistent;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;

/**
 * Robot -- a container for all of the robot hardware interfaces
 */
public class Robot
{
    // Motors
    public final DcMotor leftFront, leftRear;
    public final DcMotor rightFront, rightRear;
    public final DcMotor dunkLift, pullUp;
    public final DcMotor intakeExt;

    // PID controllers
    public final MotorController dunkLiftController;
    public final MotorController intakeExtController;

    // Servos
    public final Servo dunk;
    public final Servo hook;
    public final Servo mark;
    public final Servo intakePivot;
    public final CRServo intake;

    // Sensors
    public final IMU imu;

    public final Switch intakeLimit;
    public final Switch liftLimitDown;
    public final Switch liftLimitUp;
    public final Switch pullupLimit;

    public final RangeSensor leftRange;
    public final RangeSensor rightRange;

    // Constants
    public final double HOOK_CLOSED;
    public final double HOOK_OPEN;

    public final double dunk_min;
    public final double dunk_up;
    public final double dunk_dunk;

    public final double mark_in;
    public final double mark_out;

    public final double pivot_up;
    public final double pivot_down;

    public final int lift_up;
    public final int ext_max;  // Maximum extension position
    public final int ext_drop; // Intake extension amount at which the intake pivot should drop

    // Other
    public final Config config;

    // Internal
    private final Logger log = new Logger("Robot");

    // TODO add class for concurrent task execution
    private DataLogger turnLogger;
    private ExecutorService executor;
    private List<Future<?>> calibrationTasks = new Vector<>();

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

        intakeExt = hardwareMap.dcMotor.get("intake extension");

        // Motor controllers
        double[] dunkLiftConstants = config.getDoubleArray("dunk_lift_constants");
        dunkLiftController = new MotorController.MotorControllerFactory(dunkLift)
                                .setConstants(dunkLiftConstants)
                                .create();
        dunkLiftController.constrainPower(0, 1);
        dunkLiftController.holdStalled(false);

        double[] intakeExtConstants = config.getDoubleArray("intake_ext_constants");
        intakeExtController = new MotorController.MotorControllerFactory(intakeExt)
                                .setConstants(intakeExtConstants)
                                .create();

        // Servos
        dunk = hardwareMap.servo.get("dunk");
        hook = hardwareMap.servo.get("hook");
        mark = hardwareMap.servo.get("mark");
        intakePivot = hardwareMap.servo.get("intake pivot");
        intake = hardwareMap.crservo.get("intake");

        // Constants
        DataStorage dataStorage = new DataStorage(new File(Config.storageDir + "servo_positions.json"));
        dunk_min  = dataStorage.getDouble("dunk.Down", 0);
        dunk_up   = dataStorage.getDouble("dunk.Up",   0);
        dunk_dunk = dataStorage.getDouble("dunk.Dunk", 0);

        HOOK_CLOSED = dataStorage.getDouble("hook.Closed", 0);
        HOOK_OPEN   = dataStorage.getDouble("hook.Open",   0);

        mark_in = dataStorage.getDouble("mark.In", 0);
        mark_out = dataStorage.getDouble("mark.Out", 0);

        pivot_down = dataStorage.getDouble("intake pivot.Down", 0);
        pivot_up = dataStorage.getDouble("intake pivot.Up", 0);

        lift_up = config.getInt("lift_up", 0);
        ext_max = config.getInt("ext_max", 0);
        ext_drop = config.getInt("ext_drop", ext_max);

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
        intakeLimit = new Switch(hardwareMap.digitalChannel.get("intake limit"));
        liftLimitDown = new Switch(hardwareMap.digitalChannel.get("lower limit"));
        liftLimitUp = new Switch(hardwareMap.digitalChannel.get("upper limit"));
        pullupLimit = new Switch(hardwareMap.digitalChannel.get("pull up limit"));

        /*
        leftRange = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "left range"));
        rightRange = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "right range"));
        */

        leftRange = null;
        rightRange = null;

        // Other
        executor = Executors.newCachedThreadPool();
        turnLogger =
                new DataLogger(new File(Config.storageDir + "turnLog.bin"),
                        new DataLogger.Channel("Motor power", 0xFFFF00),
                        new DataLogger.Channel("Heading error", 0xFF0000));

        // Reverse motors as necessary
        if (config.getBoolean("lf_reverse", false)) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("lr_reverse", false)) leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("rf_reverse", false)) rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("rr_reverse", false)) rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("pl_reverse", false)) pullUp.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("dl_reverse", false)) dunkLift.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("in_reverse", false)) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("ie_reverse", false)) intakeExt.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders
        for (String name : Utils.allDeviceNames(hardwareMap.dcMotor))
        {
            DcMotor motor = hardwareMap.dcMotor.get(name);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
        dunkLiftController.close();
        intakeExtController.close();
        turnLogger.close();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Functions                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////
    // Intake pivot calibration and control

    @Deprecated
    public void initPivot() throws InterruptedException
    {
        log.w("Calling deprecated function initPivot()");
//        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakePivot.setPower(-0.4);
//        int i = 0;
//        while (!pivotLimit.pressed() && i < 2000)
//        {
//            Thread.sleep(1);
//            i++;
//        }
//        intakePivot.setPower(0);
//        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        Thread.sleep(100);
//        pivot.setPower(0.5);
//        pivot.hold(15);
    }

    public void initPivotAuto()
    {
        log.w("Calling deprecated function initPivotAuto()");
//        intakePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakePivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pivot.setPower(0.5);
//        pivot.hold(15);
    }

    public synchronized void calibrateAll()
    {
        calibrationTasks.add(executor.submit(() ->
        {
            dunk.setPosition(dunk_up);
            intakeExtController.stopHolding();
            intakeExt.setPower(-0.5);
            while (!intakeLimit.pressed())
            {
                try
                {
                    Thread.sleep(5);
                }
                catch (InterruptedException e)
                {
                    break;
                }
            }
            intakeExt.setPower(0);
            intakeExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeExt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            dunk.setPosition(dunk_min);
        }));
    }

    public synchronized boolean working()
    {
        List<Future<?>> done = new ArrayList<>();
        for (int i = 0; i < calibrationTasks.size(); i++)
        {
            if (calibrationTasks.get(i).isDone())
            {
                done.add(calibrationTasks.get(i));
            }
        }
        if (done.size() > 0) log.d("%d tasks finished", done.size());
        for (Future<?> f : done)
        {
            calibrationTasks.remove(f);
        }
        return calibrationTasks.size() > 0;
    }

    ///////////////////////////////////
    // Autonomous utilities

    public static final double ENC_PER_ROTATION_20  = 537.6; // NeveRest Orbital 20:1 motor
    public static final double ENC_PER_ROTATION_REV = 1120;  // REV Planetary 20:1 motor
    public static final double ENC_PER_INCH = ENC_PER_ROTATION_20 / 18.8496; // For 6in wheels on NeveRest motors
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

    public void turn(double angle, double power) throws InterruptedException
    {
        imu.update();
        turnTo(imu.getHeading() + angle, power);
    }

    public void turnTo(double angle, double power) throws InterruptedException
    {
        log.d("Turning to %.2f degrees", angle);
        turnLogger.startClip();
        Robot robot = Robot.instance();
        DcMotor left = robot.leftRear;
        DcMotor right = robot.rightRear;
        double kP = 0.1;
        double deadband = 3;
        for (int i = 0; (Math.abs(robot.imu.getHeading() - angle) > deadband || i < 5);)
        {
            double error = (robot.imu.getHeading() - angle);
            if (Math.abs(error) >= deadband)
            {
                left.setPower(power * Utils.constrain(error * kP, -1, 1));
                right.setPower(-power * Utils.constrain(error * kP, -1, 1));
                i = 0;
            }
            else
            {
                left.setPower(0);
                right.setPower(0);
                i++;
            }
            Thread.sleep(5);
            turnLogger.log(new double[] {left.getPower(), error});
            robot.imu.update();
        }
        left.setPower(0);
        right.setPower(0);
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

        int start = rightRear.getCurrentPosition();

        leftFront.setPower(Math.abs(power) * Math.signum(distance));
        rightFront.setPower(Math.abs(power) * Math.signum(distance));
        leftRear.setPower(Math.abs(power) * Math.signum(distance));
        rightRear.setPower(Math.abs(power) * Math.signum(distance));

        while (Math.abs(rightRear.getCurrentPosition() - start) < Math.abs(distance))
        {
            Thread.sleep(5);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


    }

}
