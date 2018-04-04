package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.util.IMUMotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.QuadrantChooser;
import org.firstinspires.ftc.teamcode.autonomous.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.teleop.util.ArmDriver;
import org.firstinspires.ftc.teamcode.teleop.util.ServoAngleFinder;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Persistent;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Stack;

/**
 * Main TeleOp control to control the {@link ArmDriver}. Most disorganized part of the code; can
 * be cleaned up
 */
@TeleOp(name="Driver Control")
public class MainTeleOp extends OpMode {

    //The arm controller
    protected ArmDriver driver;
    //Claw servo
    protected Servo claw, wrist, yaw;
    //Drive motors
    protected DcMotor base, extend;
    //Limit switch
//    protected DigitalChannel limit; //Not a digital channel anymore
    protected AnalogInput limit;

    protected IMU imu;
    //Extend motor minimum position
    protected Integer extMin = null;
    //Range of extension motor
    protected int extRange;
    //Configuration
    protected Config conf;

    protected ButtonHelper buttonHelper_1, buttonHelper_2;
    //Maximum speed of arm servos (some arbitrary unit)
    private double maxMove;
    //Maximum amount of change allowed in 200ms second
    private double maxIncrease;
    //Maximum speed of waist servo (radians per 20ms)
    private double maxRotate;

    private double[] wristRange;
    private double wrist_speed;

    private double[] yawRange;
    private double yaw_speed;

    private double[] rotateWindow = new double[10];
    private double[] extWindow = new double[10];
    private int nextWindowSlotR = 0;
    private int nextWindowSlotE = 0;
    //Whether the claw open/close button is currently being held down
    private boolean aHeld;
    //Whether the claw is open or closed
    private int clawPos = 0;
    //The amount to close the claw
    private double clawCloseAmount;
    //The amount to open the claw
    private double clawOpenAmount;
    //The amount to close the claw halfway to grab the glyph gently without destroying the servo
    private double clawGlyphAmount;

    private double l1;
    private double l2;
    private boolean robot1;

    private long start = 0;
    private long lastPress = 0;

    private Logger log;

    private volatile int quadrant;

    private int[] glyphs;
    private int activeColumn;
    private boolean rowStacking;
    private int placementStep;
    private volatile Config glyphPlacement;


    protected class MotorDriver implements Runnable {

        public static final int ROTATE = 1;
        public static final int EXTEND = 2;

        private int rotate, extension, mode;
        private Runnable onFinish;
        private volatile boolean finished = false;

        public MotorDriver(int rotate, int extend, int mode, Runnable onFinish) {
            this.rotate = rotate;
            this.extension = extend;
            this.mode = mode;
        }

        @Override
        public void run() {
            if (mode <= 0 || mode > 3) {
                finished = true;
                return;
            }
            MotorController turntable = new IMUMotorController(base, imu, conf);
            MotorController chaindrive = new MotorController(extend, conf, null, true);

            if ((mode & ROTATE) != 0) turntable.startRunToPosition(rotate);
            if ((mode & EXTEND) != 0) chaindrive.startRunToPosition(extension);

            while ((turntable.isHolding() || chaindrive.isHolding()) && !Thread.interrupted()) {
                if (motorDriverRunDetect) {
                    String holding = turntable.isHolding() ? "Running" : "Finished";
                    String position = "Position: " + turntable.getCurrentPosition() + ", Target: " +
                            turntable.getTargetPosition();
                    TelemetryWrapper.setLine(1, "Turntable: " + holding + "; " + position);
                    holding = chaindrive.isHolding() ? "Running" : "Finished";
                    position = "Position: " + chaindrive.getCurrentPosition() + ", Target: " +
                            turntable.getTargetPosition();
                    TelemetryWrapper.setLine(2, "Chain Drive: " + holding + "; " + position);
                }
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
            turntable.close();
            chaindrive.close();
            finished = true;
            if (onFinish != null) {
                onFinish.run();
            }
        }

        public boolean isFinished() {
            return finished;
        }
    }

    private MotorDriver motorDriver;
    private Thread motorDriverThread;
    private volatile boolean motorDriverRunDetect;

    public void driveMotors(int base, int extend, Runnable onFinish) {
        startDriver(new MotorDriver(base, extend, MotorDriver.ROTATE | MotorDriver.EXTEND, onFinish));
    }

    public void driveTurntable(int base, Runnable onFinish) {
        startDriver(new MotorDriver(base, 0, MotorDriver.ROTATE, onFinish));
    }

    public void driveExtend(int extend, Runnable onFinish) {
        startDriver(new MotorDriver(0, extend, MotorDriver.EXTEND, onFinish));
    }

    private void startDriver(MotorDriver driver) {
        motorDriver = driver;
        motorDriverThread = new Thread(driver, "Motor driver");
        motorDriverThread.setDaemon(true);
        motorDriverThread.start();
    }

    private void stopDriver() {
        motorDriverThread.interrupt();
    }


    @Override
    public void init() {

        //Initialize logging
        try {
            Logger.init(new File(Config.storageDir + "latest.log"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        log = new Logger("TeleOp");

        //Initialize config
        //If we're on robot 1, use robot 1 config
        if (hardwareMap.getAll(LynxModule.class).get(0).getSerialNumber().toString().equals
                ("DQ168FFD")) {
            conf = new Config("config.robot1.properties");
            robot1 = true;
            if (conf.readFailed()) {
                conf = new Config("config.properties"); // Use old config as a backup option
            }
        } else {
            conf = new Config("config.properties");
        }

        //Load configuration values
        maxMove = conf.getDouble("max_move", 0.002);
        maxIncrease = conf.getDouble("max_inc", 0.02);
        maxRotate = conf.getDouble("max_rotate_speed", 0.02);
        wristRange = conf.getDoubleArray("wrist_range");
        wrist_speed = conf.getDouble("wrist_speed", 0.01);
        yawRange = conf.getDoubleArray("yaw_range");
        yaw_speed = conf.getDouble("yaw_speed", 0.01);
        clawCloseAmount = conf.getDouble("claw_closed", 0);
        clawGlyphAmount = conf.getDouble("claw_part", 0);
        clawOpenAmount = conf.getDouble("claw_open", 0);
        l1 = conf.getDouble("l1", 1);
        l2 = conf.getDouble("l2", 1);
        extRange = conf.getInt("ext_range", Integer.MAX_VALUE/2);

        //Create button helpers
        buttonHelper_1 = new ButtonHelper(gamepad1);
        buttonHelper_2 = new ButtonHelper(gamepad2);

        //Get motors and servos from hardware map
        Servo waist = hardwareMap.servo.get("s0");
        Servo shoulder = hardwareMap.servo.get("s1");
        Servo elbow = hardwareMap.servo.get("s2");
        wrist = hardwareMap.servo.get("s4");
        claw = hardwareMap.servo.get("s3");
        yaw = hardwareMap.servo.get("s5");
        Servo colorArm = hardwareMap.servo.get("s6");
        if (conf.getDoubleArray("color_arm_positions") != null) colorArm.setPosition(conf
                .getDoubleArray("color_arm_positions")[1]);
        base = hardwareMap.dcMotor.get("base");
        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend = hardwareMap.dcMotor.get("extend");
        //extension has an encoder now!
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (!robot1) limit = hardwareMap.analogInput.get("limit");
        //Initialize arm controller
        driver = new ArmDriver(waist, shoulder, elbow, l1, l2, conf);

        //Reverse base and extension if configured to
        if (conf.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (conf.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);

        //Scale the wrist and yaw ranges
        if (wristRange != null) {
            wrist.scaleRange(wristRange[0], wristRange[1]);
        }
        if (yawRange != null) {
            yaw.scaleRange(yawRange[0], yawRange[1]);
        }

        glyphs = new int[3]; //Number of glyphs in each column
        relic_move = 0;
        placementStep = 0;
        TaskClassifyPictograph.Result findResult =
                (TaskClassifyPictograph.Result)Persistent.get("findResult");
        //Add our pre-placed glyph
        if (findResult != null) {
            if (findResult == TaskClassifyPictograph.Result.NONE
                    || findResult == TaskClassifyPictograph.Result.CENTER) {
                glyphs[1] = 1;
                activeColumn = 1;
            } else if (findResult == TaskClassifyPictograph.Result.LEFT) {
                glyphs[0] = 1;
                activeColumn = 0;
            } else if (findResult == TaskClassifyPictograph.Result.RIGHT) {
                glyphs[2] = 1;
                activeColumn = 2;
            }
        }
        rowStacking = false;

        //Initialize the IMU
        imu = (IMU)Persistent.get("imu");
        if (imu == null) {
            imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
            imu.initialize(telemetry);
        }
        imu.start();
        telemetry.clearAll();

        if (Persistent.get("quadrant") == null) {
            quadrant = -1;
            Thread chooser = new Thread(new Runnable() {
                @Override
                public void run() {
                    QuadrantChooser ch = new QuadrantChooser(telemetry);
                    quadrant = ch.chooseQuadrant();
                    glyphPlacement = new Config("glyph_quad_" + quadrant + ".txt");
                }
            });
            chooser.setDaemon(true);
            chooser.start();
        } else {
            quadrant = (int)Persistent.get("quadrant");
        }

        //Set up
        setInitialPositions();
        ServoAngleFinder.create(hardwareMap);
    }

    protected int getTurntablePosition() {
        return base.getCurrentPosition();
    }

    private void setInitialPositions() {
        claw.setPosition(clawOpenAmount);
        clawPos = 0;
        driver.moveTo(conf.getDouble("dist_init", l1+l2),
                conf.getDouble("adj_init", 0));
        driver.setWaistAngle(conf.getDouble("waist_init", 0));
        wrist.setPosition(conf.getDouble("wrist_init", 0));
        yaw.setPosition(conf.getDouble("yaw_mid", 0.5));
    }

    @Override
    public void stop() {
        if (motorDriverThread != null) stopDriver();
        imu.stop();
        Persistent.clear();
        Logger.close();
    }

    //adj, dist
    //adj, dist, waist
    //adj, dist, waist, wrist
    //adj, dist, waist, wrist, yaw
    private void moveTo(double[] armPos) {
        if (armPos == null) return;
        driver.moveTo(armPos[1], armPos[0]);
        if (armPos.length > 4) yaw.setPosition(armPos[4]);
        if (armPos.length > 3) wrist.setPosition(armPos[3]);
        if (armPos.length > 2) driver.setWaistAngle(armPos[2]);
    }

    private static final int ARM      = 1;
    private static final int WAIST    = 3;
    private static final int WRIST    = 5;
    private static final int YAW      = 9;
    private static final int ARM_FULL = WAIST | WRIST | YAW;
    private static final int ROTATE   = 16;
    private static final int EXTEND   = 32;
    private static final int BASE     = ROTATE | EXTEND;
    private static final int FULL     = ARM_FULL | BASE;

    private void moveToCoord(String property) {
        moveToCoord(property, FULL);
    }

    private void moveToCoord(String property, int mode) {
        moveToCoord(property, mode, null);
    }

    private void moveToCoord(String property, int mode, Runnable onFinish) {
        double[] coord = glyphPlacement.getDoubleArray(property);
        if (coord == null) {
            log.e("Cannot find position '%s'", property);
            return;
        }
        if ((mode & 1) != 0) {
            List<Double> co = new ArrayList<>();
            co.add(coord[1]);
            co.add(coord[0]);
            if ((mode & 2) != 0) co.add(coord[2]);
            if ((mode & 4) != 0) co.add(coord[3]);
            if ((mode & 8) != 0) co.add(coord[4]);
            Double[] d = co.toArray(new Double[0]);
            double[] o = new double[d.length];
            for (int i = 0; i < d.length; i++) {
                o[i] = d[i]; //Unboxing heck
            }
            moveTo(o);
        }
        if ((mode & 48) != 0) {
            driveMotors((int)coord[5], (int)coord[6], onFinish);
        } else if ((mode & 16) != 0) {
            driveTurntable((int)coord[5], onFinish);
        } else if ((mode & 32) != 0){
            driveExtend((int)coord[6], onFinish);
        }
    }

    private void moveToCoords(String[] properties, int[] modes) {
        if (properties.length != modes.length) {
            throw new IllegalArgumentException("Number of properties must equal number of modes!");
        }
        final List<String> props = new ArrayList<>(Arrays.asList(properties));
        Integer[] boxed = new Integer[modes.length];
        for (int i = 0; i < boxed.length; i++) {
            boxed[i] = modes[i];
        }
        final List<Integer> modeStack = new ArrayList<>(Arrays.asList(boxed));
        moveToCoord(props.remove(0), modeStack.remove(0), new Runnable() {
            @Override
            public void run() {
                if (!props.isEmpty()) {
                    moveToCoord(props.remove(0), modeStack.remove(0), this);
                }
            }
        });
    }

    private void moveToGlyphbox() {
        moveToCoord("glyph_pit");
    }

    private int relic_move;

    private void moveToRelic() {
        if (relic_move == 0) {
            relic_move = 1;
            moveToCoord("relic_0");
        } else {
            relic_move = 0;
            moveToCoord("relic_1");
        }
    }

    private void placeGlyph(int col, int lev) {
        char[] cols = {'l', 'c', 'r'};
        if (placementStep == 0) {
            placementStep = 1;
            moveToCoords(new String[] {"glyph_in", "glyph_rotated", "glyph_floating",
                                       "glyph_place_" + cols[col] + "_" + lev, "glyph_safe"},
                         new int[] {     EXTEND,     ROTATE,        ARM_FULL | EXTEND,
                                                FULL,                             ARM_FULL});
        } else {
            placementStep = 0;
            moveToGlyphbox();
        }
    }

    private void setExtendedPositions() {
        moveTo(new double[] {conf.getDouble("raise_adj", 0), conf.getDouble("raise_dist", 0)});
    }

    @Override
    public void start() {
        start = System.currentTimeMillis();
        setExtendedPositions();
    }

    /**
     * Override this method instead of loop(). Does nothing by default; super.run() not required.
     */
    public void run() {}

    @Override
    public final void loop() {
        if (quadrant < 0) {
            // Please choose a quadrant!
        } else if (motorDriver != null && !motorDriver.isFinished()) {
            if (!motorDriverRunDetect) {
                telemetry.clearAll();
                TelemetryWrapper.init(telemetry, 3);
                motorDriverRunDetect = true;
            }
            TelemetryWrapper.setLine(0, "Autonomous move in progress; press RB to stop");
            if (buttonHelper_1.pressing(ButtonHelper.right_bumper)) stopDriver();
        } else {
            if (motorDriverRunDetect) {
                telemetry.clearAll();
                motorDriverRunDetect = false;
            }
            run();
            double newDist = -(gamepad1.right_stick_y * maxMove);
            double newAngle = (gamepad1.left_stick_y * maxMove);
            if (Math.abs(Utils.sum(rotateWindow)) > maxIncrease)
                newDist = 0;
            if (Math.abs(Utils.sum(extWindow)) > maxIncrease)
                newAngle = 0;
            addToEndOfRotateWindow(newDist);
            addToEndOfExtendWindow(newAngle);
            driver.moveTo(
                    driver.getClawDistance() + newDist,
                    driver.getArmAngle() + newAngle);
            if (gamepad2.dpad_left) {
                base.setPower(0.8);
            } else if (gamepad2.dpad_right) {
                base.setPower(-0.8);
            } else {
                base.setPower(0);
            }

            wrist.setPosition(Utils.constrain(wrist.getPosition() + (wrist_speed *
                    gamepad2.right_stick_y), 0, 1));
            yaw.setPosition(Utils.constrain(yaw.getPosition() + (yaw_speed * gamepad1.right_stick_x), 0, 1));

            driver.setWaistAngle(driver.getWaistAngle() - (gamepad1.left_stick_x * maxRotate));
            //getState same as !isPressed, except for DigitalChannels (which are needed for REV sensors)
            //For AnalogInputs, getVoltage() should be around 0 when active
            //CMOS logic LOW is < 0.8V
            //if (limit.getVoltage() >= 0.8) {
            //Only allows user to go backward if the minimum switch hasn't been triggered.
            if (gamepad2.dpad_down) {
                extend.setPower(-1);
            } else {
                extend.setPower(0);
            }
            //} else {
            //    extension.setPower(0);
            //    extMin = extension.getCurrentPosition();
            //}
            if (gamepad2.dpad_up /*&& extMin != null && extension.getCurrentPosition() < extMin + extRange*/) {
                extend.setPower(1);
            } else if (extend.getPower() != -1) {
                extend.setPower(0);
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return;
            }

            //Used to be A, but that would trigger the claw when Start+A was pressed to connect gamepad1
            if (buttonHelper_1.pressing(ButtonHelper.x)) {
                if (clawPos == 0) clawPos = 1;
                else clawPos = 0;
                if (clawPos == 1)
                    claw.setPosition(clawGlyphAmount);
                else
                    claw.setPosition(clawOpenAmount);
            }

            if (buttonHelper_2.pressing(ButtonHelper.y)) {
                if (clawPos == 0) clawPos = 2;
                else clawPos = 0;
                if (clawPos == 2)
                    claw.setPosition(clawCloseAmount);
                else
                    claw.setPosition(clawOpenAmount);
            }

            if (buttonHelper_2.pressing(ButtonHelper.right_bumper)) {
                long time = System.currentTimeMillis();
                float sinceStart = (float) (time - start) / 1000f;
                float sinceLast = (float) (time - lastPress) / 1000f;
                log.i("Lap cycle trigger @ %.3f sec (%.3f sec since last press)", sinceStart, sinceLast);
                lastPress = time;
            }

            if (gamepad1.a) {
                yaw.setPosition(conf.getDouble("yaw_mid", 0));
            }

            if (buttonHelper_1.pressing(ButtonHelper.y)) {
                moveToGlyphbox();
            }

            //We don't need this in PositionCollector
            if (buttonHelper_1.pressing(ButtonHelper.b) && !(this instanceof PositionFinder)) {
                int filled = 0;
                if (rowStacking) {
                    activeColumn++;
                    activeColumn %= 3;
                    boolean found = false;
                    while (!found && filled < 3) {
                        if (glyphs[activeColumn] < 4) {
                            found = true;
                        } else {
                            activeColumn++;
                            activeColumn %= 3;
                            filled++;
                        }
                    }
                } else {
                    if (glyphs[activeColumn] == 4) {
                        activeColumn++;
                        activeColumn %= 3;
                        rowStacking = true;
                    }
                }
                if (filled < 3) {
                    placeGlyph(activeColumn, glyphs[activeColumn]);
                    glyphs[activeColumn]++;
                }
            }

            if (buttonHelper_1.pressing(ButtonHelper.right_bumper)) {
                moveToRelic();
            }

            if (gamepad1.left_bumper) {
                setInitialPositions();
            }

            telemetry.addData("Elapsed Time", Utils.elapsedTime(System.currentTimeMillis() - start));
            telemetry.addData("Claw", (clawPos == 0 ? "OPEN" : (clawPos == 1 ? "PART CLOSED" :
                    "CLOSED")));
            if (!robot1) telemetry.addData("Limit Switch", (limit.getVoltage() < 0.8) ? "PRESSED" :
                    "RELEASED");
            telemetry.addData("Arm Angle", Utils.shortFloat(driver.getArmAngle()));
            telemetry.addData("Distance", Utils.shortFloat(driver.getClawDistance()));
            telemetry.addData("Waist Position", Utils.shortFloat(driver.getWaistPos()));
            telemetry.addData("Waist Angle", Utils.shortFloat(driver.getWaistAngle()));
            telemetry.addData("Shoulder Position", Utils.shortFloat(driver.getShoulderPos()));
            telemetry.addData("Shoulder Angle", Utils.shortFloat(driver.getShoulderAngle()));
            telemetry.addData("Elbow Position", Utils.shortFloat(driver.getElbowPos()));
            telemetry.addData("Elbow Angle", Utils.shortFloat(driver.getElbowAngle()));
            telemetry.addData("Extend Position", extend.getCurrentPosition());
            telemetry.addData("Extend Minimum", extMin);
            telemetry.addData("Turntable Position", getTurntablePosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Yaw Position", yaw.getPosition());
            telemetry.addData("Running on", robot1 ? "Robot 1" : "Robot 2");
        }
    }

    private void addToEndOfRotateWindow(double value) {
        rotateWindow[nextWindowSlotR++] = value;
        //If it overflows, start writing from the beginning again
        nextWindowSlotR %= rotateWindow.length;
    }
    private void addToEndOfExtendWindow(double value) {
        extWindow[nextWindowSlotE++] = value;
        //If it overflows, start writing from the beginning again
        nextWindowSlotE %= extWindow.length;
    }

}