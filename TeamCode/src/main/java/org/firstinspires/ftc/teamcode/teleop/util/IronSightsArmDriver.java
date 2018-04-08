package org.firstinspires.ftc.teamcode.teleop.util;

import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Logger;
import org.firstinspires.ftc.teamcode.util.Utils;

import static java.lang.Math.*;

/**
 * IronSightsArmDriver -- Highly improved arm control system for driving in TeleOp and autonomously.
 */
public class IronSightsArmDriver {

    /** Fine mode -- same as MODE_FAST, but more control */
    public static final int MODE_FINE  = 1;
    /** Fast mode -- controls the arm position from the waist up */
    public static final int MODE_FAST  = 2;
    /** Gross mode -- controls the arm position and the base */
    public static final int MODE_GROSS = 3;

    // Servo coordinates for mapping

    private static double WAIST_0DEG;     // 0 deg = 0 rad
    private static double WAIST_90DEG;    // 90 deg = pi/2 rad
    private static double SHOULDER_0DEG;
    private static double SHOULDER_90DEG;
    private static double ELBOW_0DEG;
    private static double ELBOW_90DEG;
    private static double WRIST_180DEG;   // 180 deg = pi rad
    private static double WRIST_225DEG;   // 225 deg = 5*pi/4 rad

    // Arm holding the servos
    private Arm arm;
    // Motor controllers
    private MotorController base, extend;

    // Arm lengths
    private double r1, r2, r3;

    // Current position and mode
    private double x, y, z;
    private int currentMode;

    // Current angles
    private double waist_angle, shoulder_angle, elbow_angle, wrist_angle;

    // Configuration for reading constants
    private Config conf;
    // Logger
    private Logger log;

    /**
     * Initialize the arm driver
     * @param arm The arm to drive
     * @param base A motor controller to drive the base's rotation
     * @param extend A motor controller to drive the base's translation
     * @param conf A configuration file to read constants from
     */
    public IronSightsArmDriver(Arm arm, MotorController base, MotorController extend, Config conf) {
        this.arm = arm;
        this.base = base;
        this.extend = extend;
        //base.stopHolding();
        //extend.stopHolding();
        r1 = conf.getDouble("r1", 1);
        r2 = conf.getDouble("r2", 1);
        r3 = conf.getDouble("r3", 1);
        WAIST_0DEG = conf.getDouble("waist_0", 0);
        WAIST_90DEG = conf.getDouble("waist_90", 0);
        SHOULDER_0DEG = conf.getDouble("shoulder_0", 0);
        SHOULDER_90DEG = conf.getDouble("shoulder_90", 0);
        ELBOW_0DEG = conf.getDouble("elbow_0", 0);
        ELBOW_90DEG = conf.getDouble("elbow_90", 0);
        WRIST_180DEG = conf.getDouble("wrist_180", 0);
        WRIST_225DEG = conf.getDouble("wrist_225", 0);
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

    /**
     * Move the arm in Cartesian coordinate space
     * @param dx How far to move in the X direction (forward/backward)
     * @param dy How far to move in the Y direction (up/down)
     * @param dz How far to move in the Z direction (left/right)
     * @param mode How to move (any of {@link #MODE_FINE}, {@link #MODE_FAST},
     *             or {@link #MODE_GROSS})
     */
    public void drive(double dx, double dy, double dz, int mode) {
        driveAbsolute(x+dx, y+dy, z+dz, mode);
    }

    /**
     * Move the arm to an absolute position in Cartesian coordinate space
     * @param x The position in the X direction (forward/backward)
     * @param y The position in the Y direction (up/down)
     * @param z The position in the Z direction (left/right)
     * @param mode How to move (any of {@link #MODE_FINE}, {@link #MODE_FAST},
     *             or {@link #MODE_GROSS})
     */
    public void driveAbsolute(double x, double y, double z, int mode) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.currentMode = mode;
    }

    /**
     * Manually move the individual axes
     * @param base The rotating base angle, in radians (WIP)
     * @param extend The translating base distance, in inches (WIP)
     * @param waist The waist angle, in radians
     * @param shoulder The shoulder angle, in radians
     * @param elbow The elbow angle, in radians
     * @param wrist The wrist angle, in radians
     */
    public void driveManual(double base, double extend, double waist, double shoulder, double
            elbow, double wrist) {
        // Right now, we'll just ignore base and extend
        x = r1 * cos(waist) * cos(shoulder) + r2 * cos(elbow) + r3 * cos(wrist);
        y = r1 * sin(shoulder) + r2 * sin(elbow) + r3 * sin(wrist);
        z = x * sin(waist);
    }

    /**
     * Get the current mode
     * @return the current mode (one of {@link #MODE_FINE}, {@link #MODE_FAST}, or
     *         {@link #MODE_GROSS})
     */
    public int getCurrentMode() {
        return currentMode;
    }

    /*
    Move the arm (shoulder, elbow, and wrist) to the specified coordinate in 2D space
     */
    public void moveArmTo(double i, double j, double wrist) {
        //We have Cartesian coordinates (i,j) which we want to convert to polar coordinates
        //for our function
        //r4 is how far we want to go
        double r4 = sqrt(i * i + j * j);
        //t4 is the angle
        double t4 = atan2(j, i);
        //t3 is not the angle of our wrist.
        double t3 = wrist - t4 + PI;
        double t2 = elbow_angle;
        double t1 = shoulder_angle;
        int c = 0;
        double epsilon = conf.getDouble("nr_epsilon", 0.001);
        int max_iters = conf.getInt("max_iters", 100);

        //Timer
        long time = System.nanoTime();

        for (; c < max_iters; c++) {
            double[] deltas = newtonRaphson(r1, r2, r3, r4, t1, t2, t3, t4);
            t1 += deltas[0];
            t2 += deltas[1];
            if (t2 > PI) {
                double t2_last = t2;
                t2 = PI-0.1;
                t1 += abs(t2_last - t2);
            }
            if (deltas[0] < epsilon && deltas[1] < epsilon) {
                break;
            }
        }

        long elapsed = System.nanoTime() - time;
        log.d("Completed Newton-Raphson in %d iterations (%d ns)", c, elapsed);

        // We can now convert t1 and t2 into servo positions and send them to
        // shoulder and elbow respectively! :D

        setShoulderAngle(t1);
        setElbowAngle(t2);
        setWristAngle(t3);
    }

    private void setWaistAngle(double rads) {
        waist_angle = rads;
        double position = Utils.scaleRange(rads, 0, PI/2, WAIST_0DEG, WAIST_90DEG);
        arm.moveWaist(position);
    }

    private void setShoulderAngle(double rads) {
        shoulder_angle = rads;
        double position = Utils.scaleRange(rads, 0, PI/2, SHOULDER_0DEG, SHOULDER_90DEG);
        arm.moveShoulder(position);
    }

    private void setElbowAngle(double rads) {
        elbow_angle = rads;
        double position = Utils.scaleRange(rads, 0, PI/2, ELBOW_0DEG, ELBOW_90DEG);
        arm.moveElbow(position);
    }

    private void setWristAngle(double rads) {
        wrist_angle = rads;
        double position = Utils.scaleRange(rads, PI, 5.0*PI/4.0, WRIST_180DEG,
                WRIST_225DEG);
        arm.moveWrist(wrist_angle);
    }


    /*
    Returns [ dt1, dt2 ]
     */
    private double[] newtonRaphson(double r1, double r2, double r3, double r4, double t1, double t2,
                               double t3, double t4) {
        //ex = error in x direction
        //ey = error in y direction
        double ex = r1 * cos(t1) + r2 * cos(t2) - r3 * cos(t3) - r4 * cos(t4);
        double ey = r1 * sin(t1) + r2 * sin(t2) - r3 * sin(t3) - r4 * sin(t4);
        double dex1 = -r1 * sin(t1); // partial derivative of x error with respect to t1
        double dey1 =  r1 * cos(t1); // partial derivative of y error with respect to t1
        double dex2 = -r2 * sin(t2); // partial derivative of x error with respect to t2
        double dey2 =  r2 * cos(t2); // partial derivative of y error with respect to t2

        // We are solving the matrix
        //
        // [ dex1   dex2 ]   [ dt1 ]   [ -ex ]
        // [             ] * [     ] = [     ]
        // [ dey1   dey2 ]   [ dt2 ]   [ -ey ]
        //
        // for dt1 and dt2. This requires finding the inverse of the 2x2 matrix so that we
        // can rearrange the equation into
        //
        //     / [ dex1   dex2 ] \   [ -ex ]   [ dt1 ]
        // inv | [             ] | * [     ] = [     ]
        //     \ [ dey1   dey2 ] /   [ -ey ]   [ dt2 ]
        //
        //
        //   [ dey2   -dex1 ]                1                [ -ex ]   [ dt1 ]
        //   [              ] * --------------------------- * [     ] = [     ]
        //   [ -dey1   dex2 ]    dex1 * dey2 - dex2 * dey1    [ -ey ]   [ dt2 ]
        //
        // Which turns into these two formulae below
        double det = dex1 * dey2 - dex2 * dey1; // determinant of matrix
        double dt1 = (-ex * dey2 - ey * -dex1) / det; // change in angle 1
        double dt2 = (-ex * -dey1 - ey * dex2) / det; // change in angle 2
        log.d("Divide by 0 test; matrix: ");
        log.d("[ %.2f %.2f ]", dex1, dex2);
        log.d("[ %.2f %.2f ]", dey1, dey2);
        log.d(" -> determinant = %.2f", det);
        log.d("Result: dt1 = %.2f, dt2 = %.2f", dt1, dt2);
        return new double[] { dt1, dt2 };
    }
}
