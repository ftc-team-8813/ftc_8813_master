package org.firstinspires.ftc.teamcode.teleop.util;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.teleop.PositionFinder;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

import static java.lang.Math.*;

/**
 * IronSightsArmDriver -- Highly improved arm control system for driving in TeleOp and autonomously.
 */
public class IronSightsArmDriver {

    // Servo coordinates for mapping

    private static double WAIST_MIN;
    private static double WAIST_MAX;
    private static double SHOULDER_MIN;
    private static double SHOULDER_MAX;
    private static double ELBOW_MIN;
    private static double ELBOW_MAX;
    private static double WRIST_MIN;
    private static double WRIST_MAX;

    private static double WAIST_MIN_ANGLE, WAIST_MAX_ANGLE,
                          SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE,
                          ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE,
                          WRIST_MIN_ANGLE, WRIST_MAX_ANGLE;

    // Arm holding the servos
    private Arm arm;
    // Motor controllers
    //private MotorController base, extend;

    // Arm lengths
    private double r1, r2, r3;

    // Current position and mode
    private double x, y, z;
    private int currentMode;

    // Current angles
    private double waist_angle, shoulder_angle, elbow_angle, wrist_angle;
    // Shoulder 'delta angle' for balance stone tilt correction
    private double shoulder_delta;

    // Configuration for reading constants
    private Config conf;
    // Logger
    private Logger log;

    /**
     * Initialize the arm driver
     * @param arm The arm to drive
     * @param conf A configuration file to read constants from
     */
    public IronSightsArmDriver(Arm arm, Config conf) {
        this.arm = arm;
//        this.base = base;
//        this.extend = extend;
        //base.stopHolding();
        //extend.stopHolding();
        r1 = conf.getDouble("r1", 1);
        r2 = conf.getDouble("r2", 1);
        r3 = conf.getDouble("r3", 1);
        WAIST_MIN = conf.getDouble("waist_min", 0);
        WAIST_MAX = conf.getDouble("waist_max", 0);
        SHOULDER_MIN = conf.getDouble("shoulder_min", 0);
        SHOULDER_MAX = conf.getDouble("shoulder_max", 0);
        ELBOW_MIN = conf.getDouble("elbow_min", 0);
        ELBOW_MAX = conf.getDouble("elbow_max", 0);
        WRIST_MIN = conf.getDouble("wrist_min", 0);
        WRIST_MAX = conf.getDouble("wrist_max", 0);

        WAIST_MIN_ANGLE = Math.toRadians(conf.getDouble("waist_min_angle", 0));
        WAIST_MAX_ANGLE = Math.toRadians(conf.getDouble("waist_max_angle", 0));
        SHOULDER_MIN_ANGLE = Math.toRadians(conf.getDouble("shoulder_min_angle", 0));
        SHOULDER_MAX_ANGLE = Math.toRadians(conf.getDouble("shoulder_max_angle", 0));
        ELBOW_MIN_ANGLE = Math.toRadians(conf.getDouble("elbow_min_angle", 0));
        ELBOW_MAX_ANGLE = Math.toRadians(conf.getDouble("elbow_max_angle", 0));
        WRIST_MIN_ANGLE = Math.toRadians(conf.getDouble("wrist_min_angle", 0));
        WRIST_MAX_ANGLE = Math.toRadians(conf.getDouble("wrist_max_angle", 0));
        this.conf = conf;
        log = new Logger("IronSights Arm Driver");
    }

    /**
     * Get the waist angle
     * @return the waist angle, in radians
     */
    public double getWaistAngle() {
        return waist_angle;
    }

    /**
     * Get the shoulder angle
     * @return the shoulder angle, in radians
     */
    public double getShoulderAngle() {
        return shoulder_angle;
    }

    /**
     * Get the elbow angle
     * @return the elbow angle, in radians
     */
    public double getElbowAngle() {
        return elbow_angle;
    }

    /**
     * Get the wrist angle
     * @return the wrist angle, in radians
     */
    public double getWristAngle() {
        return wrist_angle;
    }

    /*
    Move the entire arm (including waist) to the specified coordinate in 3D space
     */
    public int moveArmTo(double i, double j, double k, double wrist) {
        double rArm = sqrt(i*i+k*k);
        double tArm = atan2(i, k);
        if (tArm < -PI/2) {
            rArm = -rArm;
            tArm += PI;
        }
        if (tArm > PI/2) {
            rArm = -rArm;
            tArm -= PI;
        }
        setWaistAngle(tArm);
        return moveArmTo(rArm, j, wrist);
    }

    public int moveArmCyl(double theta, double j, double k, double wrist) {
        setWaistAngle(theta);
        return moveArmTo(k, j, wrist);
    }

    public int moveArmTo(double i, double j, double wrist) {
        double[] move = calculateArm(i, j, wrist, shoulder_angle, elbow_angle, wrist_angle);
        moveArm(move);
        return (int)move[3];
    }

    public void setAdjustAngles(double shoulder) {
        shoulder_delta = shoulder;
        setWaistAngle(waist_angle);
        moveArm(shoulder_angle, elbow_angle, wrist_angle);
    }

    /**
     * Calculate the arm position from Cartesian inputs. Coordinates are two-dimensional relative
     * to the shoulder
     * @param i The position on the i-coordinate (in/out)
     * @param j The position on the j-coordinate (up/down)
     * @param t3 The requested wrist angle, counterclockwise from the i-axis
     * @param t1 The current shoulder angle
     * @param te The current elbow angle
     * @param tw The current wrist angle
     * @return The angles and exit code as follows: <code>{ shoulder, elbow, wrist, exitCode }</code>.
     *         Exit codes:
     *         <ol start=-1>
     *             <li>Stack overflow error</li>
     *             <li>The arm can reach that position</li>
     *             <li>The arm can reach that position, but the math returned a NaN</li>
     *             <li>The arm cannot reach that position, and the move should be canceled</li>
     *         </ol>
     */
    public double[] calculateArm(double i, double j, double t3, double t1, double te, double tw) {
        return calculateArm(i, j, t3, t1, te, tw, 0);
    }

    private double[] calculateArm(double i, double j, double t3, double t1, double te, double tw,
                                 int c) {
//        log.v("Stack counter = %d", c);
//        log.v("Moving to (%.4f, %.4f); wrist = %.4f", i, j, tw);
//        log.v("Current angles: shoulder=%.4f, elbow=%.4f", t1, te);
        if (c == 100) log.w("100 stack frames; is there a bug in the program?!");
        if (c == 1000) {
            log.e("1000 stack frames; exiting recursive loop");
            return new double[] {t1, te, tw, -1};
        }

        //First we want to calculate the current t4 and r4 (commented just in case we still
        // need them)
//        double r4_old_x = r1 * cos(shoulder_angle) + r2 * cos(elbow_angle) + r3 * cos(wrist_angle);
//        double r4_old_y = r1 * sin(shoulder_angle) + r2 * sin(elbow_angle) + r3 * sin(wrist_angle);
//        double r4_old = sqrt(r4_old_x*r4_old_x + r4_old_y*r4_old_y);
//        double t4_old = atan2(r4_old_y, r4_old_x);

        //We have Cartesian coordinates (i,j) which we want to convert to polar coordinates
        //for our function
        //r4 is how far we want to go
        double r4 = sqrt(i * i + j * j);
        //t4 is the angle
        double t4 = atan2(j, i);

        //t3 is our wrist angle
        //double t3 = tw + te + t1 - PI;

        double ttemp = PI + t4 - t3;

        //r5 is the distance from ground to the wrist joint
        double r5 = sqrt(r4*r4 + r3*r3 - 2*r3*r4*cos(ttemp));
        double t5 = asin((r3 * sin(t3))/r5) + t4;
        if (r5 >= r1 + r2) {
            if (abs(tw - PI) > toRadians(1)) {
//                log.v("Adjusting wrist to %.4f", t3 + PI/32);
                return calculateArm(i, j, t3 + PI/32, t1, te, tw, c+1);
            } else {
//                log.v("Unable to reach far enough; moving to straight out");
                //return new double[] { t4, PI, PI, 2 };
                return new double[] {shoulder_angle, elbow_angle, wrist_angle, 2};
            }
        }

        //By the law of cosines:
        te = acos((-r5*r5+r1*r1+r2*r2)/(2*r1*r2));
        //Now we can solve our VLE and get t1
        t1 = asin((r2 * sin(te)) / r5) + t5;

        double t2 = te + t1 - PI;
        tw = t3 - t2;

        if (tw < PI/2) {
//            log.v("Adjusting wrist so it is within physical limits (to %.4f)", t3 + PI/16);
            return calculateArm(i, j, t3 + PI/16, t1, te, tw, c+1);
        } else if (tw > 3*PI/2) {
//            log.v("Adjusting wrist so it is within physical limits (to %.4f)", t3 - PI/16);
            return calculateArm(i, j, t3 - PI/16, t1, te, tw, c+1);
        }
//        log.v("Successfully reached position");
//        log.v("New angles: shoulder=%.4f, elbow=%.4f, wrist=%.4f", t1, te, tw);
        if (Double.isNaN(t1) || Double.isNaN(te) || Double.isNaN(tw)) {
            return new double[] {shoulder_angle, elbow_angle, wrist_angle, 1};
        }
        return new double[] {t1, te, tw, 0};
    }

    private void moveArm(double[] d) {
        moveArm(d[0], d[1], d[2]);
    }

    private void moveArm(double ts, double te, double tw) {
        setShoulderAngle(ts);
        setElbowAngle(te);
        setWristAngle(tw);
    }

    private void setWaistAngle(double rads) {
        waist_angle = rads;
        if (waist_angle > PI/2) {
            waist_angle = PI/2;
        } else if (waist_angle < -PI/2) {
            waist_angle = -PI/2;
        }
        double position = Utils.scaleRange(rads, WAIST_MIN_ANGLE, WAIST_MAX_ANGLE, WAIST_MIN, WAIST_MAX);
        arm.moveWaist(position);
    }

    private void setShoulderAngle(double rads) {
        shoulder_angle = rads;
        double position = Utils.scaleRange(rads + shoulder_delta, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE,
                SHOULDER_MIN, SHOULDER_MAX);
        arm.moveShoulder(position);
    }

    private void setElbowAngle(double rads) {
        elbow_angle = rads;
        double position = Utils.scaleRange(rads, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_MIN, ELBOW_MAX);
        arm.moveElbow(position);
    }

    private void setWristAngle(double rads) {
        wrist_angle = rads;
        double position = Utils.scaleRange(rads, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, WRIST_MIN, WRIST_MAX);
        arm.moveWrist(position);
    }

    public double getWristPosition() {
        return arm.getWrist().getPosition();
    }
}
