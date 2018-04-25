package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskClassifyPictograph;
import org.firstinspires.ftc.teamcode.autonomous.util.IMUMotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;
import org.firstinspires.ftc.teamcode.util.sensors.CurrentSensor;
import org.firstinspires.ftc.teamcode.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.util.Config;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

/**
 * Joystick control for the IronSightsArmDriver
 */
public class IronSightsJoystickControl {
    //The driver for the arm
    private IronSightsArmDriver driver;
    //The gamepads
    private Gamepad gamepad1, gamepad2;
    //ButtonHelpers for the gamepads so we can have rising-edge detection
    private ButtonHelper buttons1, buttons2;
    //Current claw position
    private int clawPos;
    //Claw positions
    private double clawOpenAmount, clawGlyphAmount, clawCloseAmount;
    //Current arm position
    private double t, j, k, w;
    //Preset positions
    private double[] home, out;
    //Yaw servo position
    private double yaw;
    //Configuration
    private Config conf;
    //Arm
    private Arm arm;
    //How much to multiply the inputs by for the finesse mode (defaults to .5)
    private double finesse_gain;
    //Telemetry
    private Telemetry telemetry;
    //IMU for base rotation
    private IMU imu;
    //Motors
    private DcMotor base, extend;
    //The quadrant of the field
    private int quadrant;

    private Move currentMove;
    private boolean moveStart;
    private MotorController bc, ec;
    private Config moves;
    private int moveStep;
    private boolean rowStacking;
    private int[] glyphs;
    private int activeColumn;


    private long start = 0;
    private long lastPress = 0;

    private CurrentSensor currentSensor;
    private double floatDistance;
    private boolean floorTest;
    private long floorTestTime;

    private boolean isPositionFinder;

    private long lbPress;

    private double bbRadius;

    private Logger log;

    private boolean relicMode;
    private int relicStep;
    private boolean stepTrigger;
    private long relicWaitTime;

    public IronSightsJoystickControl(Gamepad gamepad1, Gamepad gamepad2, Arm arm, Config conf,
                                     Telemetry telemetry, DcMotor base, DcMotor extend, IMU imu, int quadrant,
                                     TaskClassifyPictograph.Result findResult, boolean isPositionFinder) {
        log = new Logger("IronSights Joystick Driver");
        this.telemetry = telemetry;
        this.base = base;
        this.extend = extend;
        this.imu = imu;
        bc = new IMUMotorController(base, imu, conf);
        ec = new MotorController(extend, conf);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        buttons1 = new ButtonHelper(gamepad1);
        buttons2 = new ButtonHelper(gamepad2);
        driver = new IronSightsArmDriver(arm, conf);
        this.arm = arm;
        this.conf = conf;
        this.quadrant = quadrant;
        this.isPositionFinder = isPositionFinder;
        moves = new Config("ironSightsPresets_" + quadrant + ".txt");

        glyphs = new int[3];
        moveStep = 0;
        rowStacking = false;
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

        //Read configuration values
        finesse_gain = conf.getDouble("finesse_gain", 0.5);
        home = conf.getDoubleArray("ironSights_home");
        if (home == null) home = new double[] {0, 3, -5, PI, 0, conf.getDouble("yaw_mid", 0.5), 0, 0};
        out = conf.getDoubleArray("ironSights_out");
        if (out == null) out = new double[] {0, 9, 20, PI, 0, conf.getDouble("yaw_mid", 0.5), 0, 0};
        clawCloseAmount = conf.getDouble("claw_closed", 0);
        clawGlyphAmount = conf.getDouble("claw_part", 0);
        clawOpenAmount = conf.getDouble("claw_open", 0);

        relicMode = false;
        relicStep = 0;
        stepTrigger = true;

        t = home[0];
        j = home[1];
        k = home[2];
        w = home[3];
        yaw = conf.getDouble("yaw_mid", 0.5);

        bbRadius = (conf.getDouble("r1", 18) + conf.getDouble("r2", 18)) / Math.sqrt(2) + 3;

        //Move to home
        currentMove = new Move(home, Move.FULL);
        moveStart = true;
        currentMove.moveTo(this, driver, bc, ec);
        relicWaitTime = 0;
        arm.moveYaw(yaw);
        arm.openClaw();
    }

    public void start() {
        start = System.currentTimeMillis();
        t = out[0];
        j = out[1];
        k = out[2];
        w = out[3];
        driver.moveArmTo(t, j, k, w);
    }

    public double i() {
        return t;
    }

    public double j() {
        return j;
    }

    public double k() {
        return k;
    }

    public double w() {
        return w;
    }

    public int claw() {
        return clawPos;
    }

    public double clawPosition() {
        return arm.getClaw().getPosition();
    }

    public double yaw() {
        return yaw;
    }

    public int base() {
        return bc.getCurrentPosition();
    }

    public int extend() {
        return ec.getCurrentPosition();
    }

    public Arm arm() {
        return arm;
    }

    private void moveToManual(double[] move) {
        arm.moveWaist(move[0]);
        arm.moveShoulder(move[1]);
        arm.moveElbow(move[2]);
        double[] clawPos = {clawOpenAmount, clawGlyphAmount, clawCloseAmount};
        arm.moveClaw(clawPos[(int)move[3]]);
        arm.moveWrist(move[4]);
        arm.moveYaw(move[5]);
        bc.startRunToPosition((int)move[6]);
        ec.startRunToPosition((int)move[7]);
    }

    public void loop() {
        if (relicMode) { //Relic placement -- our very VERY special case
            switch (relicStep) {
                case 0:
                    if (stepTrigger) {
                        stepTrigger = false;
                        double[] move = moves.getDoubleArray("relic_out");
                        moveToManual(move);
                    }
                    if (!bc.isHolding() && !ec.isHolding()) {
                        if (relicWaitTime == 0) {
                            relicWaitTime = System.currentTimeMillis();
                        } else if (System.currentTimeMillis() > relicWaitTime + 1000) {
                            stepTrigger = true;
                            relicStep++;
                        }
                    }
                    break;
                case 1:
                    if (stepTrigger) {
                        stepTrigger = false;
                        double[] move = moves.getDoubleArray("relic_extend");
                        moveToManual(move);
                    }
                    if (!bc.isHolding() && !ec.isHolding() && !gamepad1.right_bumper) {
                        stepTrigger = true;
                        relicStep++;
                    }
                    break;
                case 2:
                    if (stepTrigger) {
                        stepTrigger = false;
                        double[] move = moves.getDoubleArray("relic_place");
                        moveToManual(move);
                    }
                    if (!bc.isHolding() && !ec.isHolding()) {
                        if (relicWaitTime == 0) {
                            relicWaitTime = System.currentTimeMillis();
                        } else if (System.currentTimeMillis() > relicWaitTime + 200) {
                            stepTrigger = true;
                            relicStep++;
                        }
                    }
                    break;
                case 3:
                    telemetry.addData("CONTROLLER 2", "Press RB when finished");
                    double wrist_inc = -gamepad2.right_stick_y / 75;
                    double yaw_inc = -gamepad2.left_stick_x / 75;
                    double w = arm.getWrist().getPosition();
                    double y = arm.getYaw().getPosition();
                    w += wrist_inc;
                    y += yaw_inc;
                    if (w > 1) w = 1;
                    if (w < 0) w = 0;
                    if (y > 1) y = 1;
                    if (y < 0) y = 0;
                    arm.moveWrist(w);
                    arm.moveYaw(y);
                    if (buttons2.pressing(ButtonHelper.y)) {
                        if (clawPos == 0) clawPos = 2;
                        else clawPos = 0;
                        if (clawPos == 2)
                            arm.moveClaw(clawCloseAmount);
                        else
                            arm.moveClaw(clawOpenAmount);
                    }
                    if (gamepad1.right_bumper) {
                        stepTrigger = true;
                        relicStep++;
                    }
                    if (gamepad2.dpad_left) {
                        base.setPower(0.8); //TODO make configurable
                    } else if (gamepad2.dpad_right) {
                        base.setPower(-0.8);
                    } else {
                        base.setPower(0);
                    }
                    if (gamepad2.dpad_up) {
                        extend.setPower(1); //TODO make configurable
                    } else if (gamepad2.dpad_down) {
                        extend.setPower(-1);
                    } else {
                        extend.setPower(0);
                    }
                    break;
                case 4:
                    currentMove =        new Move(moves.getDoubleArray("relic_up"), Move.ARM ).setWaitTime(500);
                    currentMove
                            .setNextMove(new Move(moves.getDoubleArray("home"    ), Move.FULL_MOVE)).setWaitTime(0);
                    relicMode = false;
                    break;
            }
        } else if (currentMove != null) {
            if (!moveStart) {
                moveStart = true;
                telemetry.clear();
                telemetry.addData("", "Press RB to stop");
                currentMove.moveTo(this, driver, bc, ec);
            }
            if (gamepad1.right_bumper) {
                currentMove.stop();
            }
            if (!currentMove.driving()) {
                currentMove = null;
            }
        } else {
            if (moveStart) {
                moveStart = false;
                telemetry.clear();
            }
            double dx = -gamepad1.right_stick_x * PI/64;
            double dy = gamepad1.left_stick_y / 1.5;
            double dz = -gamepad1.right_stick_y;
            double dw = -gamepad2.right_stick_y / 10;
            double dyw = -gamepad2.left_stick_x;
            if (gamepad1.right_bumper) {
                dx *= finesse_gain;
                dy *= finesse_gain;
                dz *= finesse_gain;
                dw *= finesse_gain;
            }
            //imu.getHeading() is clockwise, in degrees, from 45 degrees or 135 degrees, depending on
            //which balance stone we're on
            double start_angle = (quadrant % 2 == 0) ? 45 : 135;
            double base_angle = toRadians(start_angle - imu.getHeading());
            //Uncomment these lines and comment the ones following it to turn off parallel-to-walls joystick control:
            double dt = dx;
            double dk = dz;
            //double dt = dx * cos(base_angle) + dz * sin(base_angle);
            //double dk = dx * sin(base_angle) + dz * cos(base_angle);
            double dj = dy;
            if (Math.abs(t + dt) < PI/2) t += dt;
            else t = PI/2 * Math.signum(t);
            if (Math.abs(j + dj) < bbRadius) j += dj;
            else j = bbRadius * Math.signum(j);
            if (Math.abs(k + dk) < bbRadius) k += dk;
            else k = bbRadius * Math.signum(k);
            w += dw;
            yaw += dyw / 50; //possibly make this constant configurable?

            if (w > 3 * PI / 2) {
                w = 3 * PI / 2;
            } else if (w < PI / 2) {
                w = PI / 2;
            }
            if (yaw > 1) {
                yaw = 1;
            } else if (yaw < 0) {
                yaw = 0;
            }
            driver.setAdjustAngles(-Math.toRadians(imu.getPitch()));
            int code = driver.moveArmCyl(t, j, k, w);
            if (code >= 2) {
                t -= dt;
                j -= dj;
                k -= dk;
                w -= dw;
            }
            arm.moveYaw(yaw);
            if (buttons1.pressing(ButtonHelper.x)) {
                if (clawPos == 0) clawPos = 1;
                else clawPos = 0;
                if (clawPos == 1)
                    arm.moveClaw(clawGlyphAmount);
                else
                    arm.moveClaw(clawOpenAmount);
            }

            if (buttons2.pressing(ButtonHelper.y)) {
                if (clawPos == 0) clawPos = 2;
                else clawPos = 0;
                if (clawPos == 2)
                    arm.moveClaw(clawCloseAmount);
                else
                    arm.moveClaw(clawOpenAmount);
            }

            if (gamepad2.dpad_left) {
                base.setPower(0.8); //TODO make configurable
            } else if (gamepad2.dpad_right) {
                base.setPower(-0.8);
            } else {
                base.setPower(0);
            }

            if (gamepad2.dpad_up) {
                extend.setPower(1); //TODO make configurable
            } else if (gamepad2.dpad_down) {
                extend.setPower(-1);
            } else {
                extend.setPower(0);
            }

            if (buttons1.pressing(ButtonHelper.left_bumper)) {
                if (lbPress == 0) {

                }
                if (System.currentTimeMillis() - lbPress < 500) {
                    currentMove = new Move(home, Move.FULL);
                } else {
                    lbPress = System.currentTimeMillis();
                    currentMove = new Move(out, Move.FULL);
                }
            }

//            if (floorTest && System.currentTimeMillis() > floorTestTime + 1000) {
//                floorTest = false;
//                if (currentSensor.getCurrent() > 1.5) {
//                    //We have a block
//                    currentMove = new Move(moves.getDoubleArray(""))
//                }
//            }

            if (gamepad1.b && !isPositionFinder) {
                try {
                    int filled = 0;
                    if (moveStep == 0) {
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
                        }
                    } else {
                        placeGlyph(0, 0); // we don't need x,y
                    }
                } catch (IllegalArgumentException e) {
                    log.e(e);
                }
                //Place da glyph
            }

            if (gamepad1.y) {
                if (relicStep == 0) {
                    relicStep = 1;
                    currentMove = new Move(moves.getDoubleArray("relic_spin"), Move.FULL_MOVE).setWaitTime(500);
                    currentMove.setNextMove(new Move(moves.getDoubleArray("relic_grab"), Move.FULL_MOVE)).setWaitTime(500);
                } else {
                    relicStep = 0;
                    currentMove = new Move(moves.getDoubleArray("relic_lift"), Move.FULL_MOVE).setWaitTime(500);
                    currentMove
                            .setNextMove(new Move(moves.getDoubleArray("relic_turn"), Move.FULL_MOVE)).setWaitTime(500)
                            .onFinish(new Runnable() {
                                @Override
                                public void run() {
                                    try {
                                        Thread.sleep(500);
                                    } catch (InterruptedException e) {
                                        return;
                                    }
                                    relicMode = true;
                                    arm.moveClaw(clawCloseAmount);
                                }
                            });
                }
            }

            if (gamepad2.a) {
                yaw = conf.getDouble("yaw_mid", 0.5);
            }

            telemetry.addData("Elapsed Time", Utils.elapsedTime(System.currentTimeMillis() - start));
            telemetry.addData("t", t);
            telemetry.addData("j", j);
            telemetry.addData("k", k);
            telemetry.addData("w", w);
            telemetry.addData("Waist", driver.getWaistAngle());
            telemetry.addData("Shoulder", driver.getShoulderAngle());
            telemetry.addData("Elbow", driver.getElbowAngle());
            telemetry.addData("Wrist", driver.getWristAngle());
            telemetry.addData("IMU Heading", imu.getHeading());
        }
    }

    public void placeGlyph(int x, int y) {
        if (moveStep == 0) {
            //moveStep = 1;
            currentMove =        new Move(moves.getDoubleArray("return_to_pit"       ), Move.FULL_MOVE         ).setWaitTime(500);
            currentMove
                    .setNextMove(new Move(moves.getDoubleArray("to_cryptobox_pre"    ), Move.FULL_MOVE        )).setWaitTime(200)
                    .setNextMove(new Move(moves.getDoubleArray("to_cryptobox"        ), Move.FULL_MOVE        )).setWaitTime(0)
                    .setNextMove(new Move(moves.getDoubleArray("glyph_" + x + "_" + y), Move.FULL             )).setWaitTime(800)
                    .onFinish(new Runnable() {
                        @Override
                        public void run() {
                            glyphs[activeColumn]++;
                        }
                    });
        }
        /* else {
            moveStep = 0;
            currentMove =        new Move(moves.getDoubleArray("drop_glyph"          ), Move.CLAW              ).setWaitTime(100);
            currentMove
                    .setNextMove(new Move(moves.getDoubleArray("float_" + x          ), Move.J | Move.K)).setWaitTime(800)
                    .setNextMove(new Move(moves.getDoubleArray("return_to_pit"       ), Move.FULL_MOVE        )).setWaitTime(0);
        }*/
    }

    public void stop() {
        imu.stop();
        if (currentMove != null) currentMove.stop();
        bc.close();
        ec.close();
    }

    public static class Move {
        //Modes
        //Tell the Move which axes to change
        private static final int I         = 1;
        private static final int J         = 2;
        private static final int K         = 4;
        private static final int WRIST     = 8;
        private static final int CLAW      = 16;
        private static final int YAW       = 32;
        private static final int ARM       = I | J | K | WRIST;
        private static final int ARM_FULL  = ARM | CLAW | YAW;
        private static final int BASE      = 64;
        private static final int EXTEND    = 128;
        private static final int BASE_FULL = BASE | EXTEND;
        private static final int FULL_MOVE = ARM | BASE_FULL;
        private static final int FULL      = ARM_FULL | BASE_FULL;

        private int mode;
        //Order:
        //[ t j k wrist claw yaw base extend ]
        //NOTE: claw is an int (0, 1, 2) -> (open, part closed, closed)
        private double[] data;
        private Thread driver;
        private Move next;
        private Runnable onFinish;
        private long nextTime;

        public static Move arm(double i, double j, double k, double wrist) {
            double[] data = {i, j, k, wrist, 0, 0, 0, 0};
            return new Move(data, ARM);
        }

        public static Move base(double turn, double ext) {
            double[] data = {0, 0, 0, 0, 0, 0, turn, ext};
            return new Move(data, BASE_FULL);
        }

        public static Move armFull(double i, double j, double k, double wrist, double claw, double yaw) {
            double[] data = {i, j, k, wrist, claw, yaw, 0, 0};
            return new Move(data, ARM_FULL);
        }

        public Move(double[] data, int mode) {
            this.mode = mode;
            this.data = data;
            if (data == null) throw new IllegalArgumentException("Null data for move");
            if (data.length < 8) throw new IllegalArgumentException("Data must have length >=8");
        }

        public Move setNextMove(Move next) {
            this.next = next;
            return next;
        }

        public Move onFinish(Runnable onFinish) {
            this.onFinish = onFinish;
            return this;
        }

        public Move setWaitTime(long millis) {
            nextTime = millis;
            return this;
        }

        public void moveTo(final IronSightsJoystickControl ctrl, final IronSightsArmDriver driver,
                           final MotorController base, final MotorController extend) {
            if ((mode & I) != 0) {
                ctrl.t = data[0];
            }
            if ((mode & J) != 0) {
                ctrl.j = data[1];
            }
            if ((mode & K) != 0) {
                ctrl.k = data[2];
            }
            if ((mode & WRIST) != 0) {
                ctrl.w = data[3];
            }
            if ((mode & CLAW) != 0) {
                ctrl.clawPos = (int)data[4];
            }
            if ((mode & YAW) != 0) {
                ctrl.yaw = data[5];
            }
            int dMode = 0;
            double[] driverData = new double[2];
            if ((mode & BASE) != 0) {
                dMode |= MotorDriver.BASE;
                driverData[0] = data[6];
            }
            if ((mode & EXTEND) != 0) {
                dMode |= MotorDriver.EXT;
                driverData[1] = data[7];
            }
            driver.moveArmTo(ctrl.t, ctrl.j, ctrl.k, ctrl.w);
            double[] clawPos = {ctrl.clawOpenAmount, ctrl.clawGlyphAmount, ctrl.clawCloseAmount};
            ctrl.arm.moveClaw(clawPos[ctrl.clawPos]);
            ctrl.arm.moveYaw(ctrl.yaw);
            this.driver = new Thread(new MotorDriver(driverData, dMode, base, extend, new Runnable() {
                @Override
                public void run() {
                    //Run the next move using the motor driver
                    try {
                        Thread.sleep(nextTime);
                    } catch (InterruptedException e) {
                        return;
                    }
                    if (next != null) next.moveTo(ctrl, driver, base, extend);
                    if (onFinish != null) onFinish.run();
                }
            }));
            this.driver.start();
        }

        public boolean driving() {
            return (driver != null && driver.isAlive()) || (next != null && next.driving());
        }

        public void stop() {
            if (driver != null) driver.interrupt();
            if (next != null) next.stop();
        }
    }

    public static class MotorDriver implements Runnable {

        public static final int BASE = 1;
        public static final int EXT  = 2;
        private int mode;
        private double[] data;
        private MotorController bc, ec;
        private Runnable next;

        public MotorDriver(double[] data, int mode, MotorController base, MotorController extend, Runnable next) {
            this.mode = mode;
            this.data = data;
            if (data == null) throw new IllegalArgumentException("Null data to motor driver");
            if (data.length < 2) throw new IllegalArgumentException("Data must have length >=2");
            bc = base;
            ec = extend;
            this.next = next;
        }

        @Override
        public void run() {
            if ((mode & 1) != 0) {
                bc.startRunToPosition((int)data[0]);
            }
            if ((mode & 2) != 0) {
                ec.startRunToPosition((int)data[1]);
            }
            while (bc.isHolding() || ec.isHolding()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    break;
                }
            }
            // In case the thread is interrupted
            bc.stopHolding();
            ec.stopHolding();
            if (next != null && !Thread.interrupted()) next.run();
        }
    }
}
