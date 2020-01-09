package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.actuators.Claw;
import org.firstinspires.ftc.teamcode.common.actuators.Drivetrain;
import org.firstinspires.ftc.teamcode.common.actuators.FoundationHook;
import org.firstinspires.ftc.teamcode.common.actuators.Intake;
import org.firstinspires.ftc.teamcode.common.actuators.IntakeLinkage;
import org.firstinspires.ftc.teamcode.common.actuators.Lift;
import org.firstinspires.ftc.teamcode.common.actuators.MotorArm;
import org.firstinspires.ftc.teamcode.common.motor_control.PIDMotor;
// import org.firstinspires.ftc.teamcode.common.actuators.SwerveWheel;
import org.firstinspires.ftc.teamcode.common.sensors.AMSEncoder;
import org.firstinspires.ftc.teamcode.common.sensors.IMU;
import org.firstinspires.ftc.teamcode.common.sensors.RangeSensor;
import org.firstinspires.ftc.teamcode.common.sensors.Switch;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.GlobalDataLogger;
import org.firstinspires.ftc.teamcode.common.util.Logger;
import org.firstinspires.ftc.teamcode.common.util.RevHubLED;
import org.firstinspires.ftc.teamcode.common.util.Utils;

import java.io.File;

/**
 * Robot -- a container for all of the robot hardware interfaces
 */
public class Robot
{
    // Motors
    public final Intake intake;

    // PID-controlled motors

    // Servos
    public final FoundationHook foundationhook;
    
    // Actuators
    public final Drivetrain drivetrain;
    public final Lift slide;
    public final Claw claw;
    public final IntakeLinkage intakelinkage;
    public final MotorArm newarm;

    // Sensors
    public final Switch bottomlimit;
    public final Switch backSwitch;
    public final IMU imu;
    
    public final AMSEncoder fwdEnc;
    public final AMSEncoder strafeEnc;
    
    public final RangeSensor leftRange;
    public final RangeSensor centerRange;
    public final RangeSensor rightRange;


    // Constants

    // Other
    public final Config config;
    public final HardwareMap hardwareMap;
    
    public final RevHubLED leftLed, rightLed;

    // Internal
    private final Logger log = new Logger("Robot");

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
        instance = this; // Pre-set the instance so other classes can see this
        this.hardwareMap = hardwareMap;
        this.config = config;
        // Motors
        DcMotor leftIntake = hardwareMap.dcMotor.get("l intake");
        DcMotor rightIntake = hardwareMap.dcMotor.get("r intake");
        intake = new Intake(leftIntake, rightIntake);


        // PID controllers


        // Servos
        DataStorage servo_positions = new DataStorage(new File(Config.storageDir + "servo_positions.json"));
    
        Servo hookL = hardwareMap.servo.get("hook l");
        Servo hookR = hardwareMap.servo.get("hook r");
        foundationhook = new FoundationHook(hookL, hookR, servo_positions);

        Servo intake_l = hardwareMap.servo.get("intake l");
        Servo intake_r = hardwareMap.servo.get("intake r");
        
        intakelinkage = new IntakeLinkage(intake_l, intake_r, servo_positions);

        // Actuators
        DcMotor slidemotor = hardwareMap.dcMotor.get("slide lift");
        slidemotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DigitalChannel bottomswitch = hardwareMap.digitalChannel.get("bottom limit");
        PIDMotor lift = new PIDMotor(slidemotor);
        bottomlimit = new Switch(bottomswitch);
        slide = new Lift(lift, bottomlimit);

        DcMotor motorArm = hardwareMap.dcMotor.get("motor arm");
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        DigitalChannel backLimit = hardwareMap.digitalChannel.get("back limit");
        backSwitch = new Switch(backLimit);
        newarm = new MotorArm(motorArm, backSwitch);


        imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        
        // fwdEnc = hardwareMap.get(AMSEncoder.class, "fwd enc");
        // strafeEnc = hardwareMap.get(AMSEncoder.class, "strafe enc");
        fwdEnc = null;
        strafeEnc = null;
        
        // AccelMotor edition
        /*
        drivetrain = new Drivetrain(new PIDMotor(new AccelMotor(hardwareMap.dcMotor.get("lf"))),
                                    new PIDMotor(new AccelMotor(hardwareMap.dcMotor.get("rf"))),
                                    new PIDMotor(new AccelMotor(hardwareMap.dcMotor.get("lb"))),
                                    new PIDMotor(new AccelMotor(hardwareMap.dcMotor.get("rb"))), imu);
        */
        
        // Non-AccelMotor drivetrain
        drivetrain = new Drivetrain(new PIDMotor(hardwareMap.dcMotor.get("lf")),
                                    new PIDMotor(hardwareMap.dcMotor.get("rf")),
                                    new PIDMotor(hardwareMap.dcMotor.get("lb")),
                                    new PIDMotor(hardwareMap.dcMotor.get("rb")), imu, fwdEnc, strafeEnc);
    
        claw = new Claw(hardwareMap.servo.get("claw"), servo_positions);
        
        // Swerve wheels
        
        
        // Sensors
        // leftRange = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "l distance"));
        leftRange = null;
        centerRange = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "c distance"));
        // rightRange = new RangeSensor(hardwareMap.get(Rev2mDistanceSensor.class, "r distance"));
        rightRange = null;
        
        // Constants

        
        // Other
        leftLed = new RevHubLED(hardwareMap.get(LynxModule.class, "Expansion Hub left"));
        rightLed = new RevHubLED(hardwareMap.get(LynxModule.class, "Expansion Hub right"));
        
        // Reverse motors as necessary
        
        
        // Logging
        /*
        GlobalDataLogger.instance().addChannel("Encoder abs. distance (fwd)", () -> "" + fwdEnc.getAbsoluteAngle());
        GlobalDataLogger.instance().addChannel("Encoder distance (fwd)", () -> "" + fwdEnc.getAngle());
        GlobalDataLogger.instance().addChannel("Encoder abs. distance (strafe)", () -> "" + strafeEnc.getAbsoluteAngle());
        GlobalDataLogger.instance().addChannel("Encoder distance (strafe)", () -> "" + strafeEnc.getAngle());
        */
        
        // GlobalDataLogger.instance().addChannel("Left Range", () -> String.format("%.4f", leftRange.getDistance()));
        GlobalDataLogger.instance().addChannel("Center Range", () -> String.format("%.4f", centerRange.getDistance()));
        // GlobalDataLogger.instance().addChannel("Right Range", () -> String.format("%.4f", rightRange.getDistance()));

        
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
        instance = new Robot(hardwareMap, config); // Redundant assignment but looks better
        return instance;
    }

    public static Robot instance()
    {
        return instance;
    }

    public void uninitialize()
    {
        // Stop all motors

        // Stop external threads and close open files (if any) here
        if (imu != null) imu.stop();
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
//  Functions                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////



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
        // TODO add robot specific IMU control
//        imu.update();
//        turnTo(imu.getHeading() + angle, power);
    }

    public void turnTo(double angle, double power) throws InterruptedException
    {
        // TODO needs IMU and motors
//        log.d("Turning to %.2f degrees", angle);
//        turnLogger.startClip();
//        Robot robot = Robot.instance();
//        DcMotor left = robot.leftRear;
//        DcMotor right = robot.rightRear;
//        double kP = 0.1;
//        double deadband = 3;
//        for (int i = 0; (Math.abs(robot.imu.getHeading() - angle) > deadband || i < 5);)
//        {
//            double error = (robot.imu.getHeading() - angle);
//            if (Math.abs(error) >= deadband)
//            {
//                left.setPower(power * Utils.constrain(error * kP, -1, 1));
//                right.setPower(-power * Utils.constrain(error * kP, -1, 1));
//                i = 0;
//            }
//            else
//            {
//                left.setPower(0);
//                right.setPower(0);
//                i++;
//            }
//            Thread.sleep(5);
//            turnLogger.log(new double[] {left.getPower(), error});
//            robot.imu.update();
//        }
//        left.setPower(0);
//        right.setPower(0);
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
        // TODO needs motors
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int start = rightRear.getCurrentPosition();
//
//        leftFront.setPower(Math.abs(power) * Math.signum(distance));
//        rightFront.setPower(Math.abs(power) * Math.signum(distance));
//        leftRear.setPower(Math.abs(power) * Math.signum(distance));
//        rightRear.setPower(Math.abs(power) * Math.signum(distance));
//
//        while (Math.abs(rightRear.getCurrentPosition() - start) < Math.abs(distance))
//        {
//            Thread.sleep(5);
//        }
//
//        leftFront.setPower(0);
//        leftRear.setPower(0);
//        rightFront.setPower(0);
//        rightRear.setPower(0);


    }

}
