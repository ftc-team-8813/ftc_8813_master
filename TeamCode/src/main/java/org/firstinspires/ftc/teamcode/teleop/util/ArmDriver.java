package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Utils;
import static java.lang.Math.*;

/**
 * ArmDriver - Drive the arm servos based on controller input.
 */

public class ArmDriver {

    // Constants -- should be migrated to config?

    /** Maximum servo angle (pos = 1) in radians */
    public static final double SERVO_MAX_ANGLE = 7.04614352305139340626;
    /** Position of shoulder servo when horizontal (0 radians) */
    public static final double SERVO_ANGLE_H_S = 0.4622;
    /** Position of elbow servo when horizontal (0 radians) */
    public static final double SERVO_ANGLE_H_E = 0.5042;
    /** Position of shoulder servo when vertical (pi/2 radians) */
    public static final double SERVO_ANGLE_V_S = 0.2589;
    /** Position of elbow servo when vertical (pi/2 radians) */
    public static final double SERVO_ANGLE_V_E = 0.4985;
    /** Length from the shoulder to the elbow (assumed to be equal to LENGTH_E_C, but this is
     * definitely wrong */
    public static final double LENGTH_S_E      = 1;
    /** Length from the elbow to the claw (definition of the cubit; should always equal 1 :D ) */
    public static final double LENGTH_E_C      = 1;

    // Sub-classes

    /**
     * ArmPos - Describes the position of the arm [ Not used; might delete ]
     */
    private static class ArmPos {
        private final double w, s, e;
        public ArmPos(double waist, double shoulder, double elbow) {
            this.w = waist;
            this.s = shoulder;
            this.e = elbow;
        }

        public double getWaistRadians() {
            return w;
        }

        public double getShoulderRadians() {
            return s;
        }

        public double getElbowRadians() {
            return e;
        }

    }

    // Fields

    //Servos
    private final Servo ws, ss, es;
    //Points of origin for xy to angle calculation (never change)
    private final double x=0, y=0, x2=0, y2=0;
    //Current position in cartesian coordinates
    private double cx, cy;
    //Current servo positions in servo angles (0-1)
    private double[] currentPositions = new double[3];
    //Current servo positions in radians
    private double[] currentPositionsR = new double[3];

    // Constructors

    /**
     * Construct the arm driver. Does not attempt to move the servos.
     * @param waist The waist servo
     * @param shoulder The shoulder servo
     * @param elbow The elbow servo
     */
    public ArmDriver(Servo waist, Servo shoulder, Servo elbow) {
        ws = waist;
        ss = shoulder;
        es = elbow;
    }

    // Methods

    // Direct drive

    /**
     * Set the angle of the waist servo in radians
     * @param radians The new angle to target
     */
    public void setWaistAngle(double radians) {
        currentPositions[0] =
                Utils.scaleRange(radians, 0, SERVO_MAX_ANGLE, 0, 1);
        currentPositionsR[0] = radians;
        calculateXYandMove();
    }

    /**
     * Set the angle of the shoulder servo in radians
     * @param radians The new angle to target
     */
    public void setShoulderAngle(double radians) {
        currentPositions[1] =
                Utils.scaleRange(radians, 0, PI/2, SERVO_ANGLE_H_S, SERVO_ANGLE_V_S);
        currentPositionsR[1] = radians;
        calculateXYandMove();
    }

    /**
     * Set the angle of the elbow servo in radians
     * @param radians The new angle to target
     */
    public void setElbowAngle(double radians) {
        currentPositions[2] =
                Utils.scaleRange(radians, 0, PI/2, SERVO_ANGLE_H_E, SERVO_ANGLE_V_E);
        currentPositionsR[2] = radians;
        calculateXYandMove();
    }

    //Getters

    /**
     * Get the previously set (or calculated) x position of the claw.
     * @return The current target x position of the claw
     */
    public double getClawX() {
        return cx;
    }

    /**
     * Get the previously set (or calculated) y position of the claw.
     * @return The current target y position of the claw
     */
    public double getClawY() {
        return cy;
    }

    /**
     * Get the previously set (or calculated) position (servo angle) of the waist servo
     * @return The current target servo position of the waist
     */
    public double getWaistPos() {
        return currentPositions[0];
    }

    /**
     * Get the previously set (or calculated) angle (radians) of the waist servo
     * @return The current target servo position of the waist
     */
    public double getWaistAngle() {
        return currentPositionsR[0];
    }

    /**
     * Get the previously set (or calculated) position (servo angle) of the shoulder servo
     * @return The current target servo position of the shoulder
     */
    public double getShoulderPos() {
        return currentPositions[1];
    }

    /**
     * Get the previously set (or calculated) angle (radians) of the shoulder servo
     * @return The current target servo position of the shoulder
     */
    public double getShoulderAngle() {
        return currentPositionsR[1];
    }

    /**
     * Get the previously set (or calculated) position (servo angle) of the elbow servo
     * @return The current target servo position of the elbow
     */
    public double getElbowPos() {
        return currentPositions[2];
    }

    /**
     * Get the previously set (or calculated) angle (radians) of the shoulder servo
     * @return The current target servo position of the shoulder
     */
    public double getElbowAngle() {
        return currentPositionsR[2];
    }

    private void calculateXYandMove() {
        double sr = currentPositionsR[1];
        double er = currentPositionsR[2];
        double x1 = Math.cos(sr);
        double y1 = Math.sin(sr);
        double x2 = Math.cos(er);
        double y2 = Math.sin(er);
        cx = x1 + x2;
        cy = y1 + y2;
        ws.setPosition(currentPositions[0]);
        ss.setPosition(currentPositions[1]);
        es.setPosition(currentPositions[2]);
    }

    /**
     * Move the waist, shoulder, and elbow servos in one operation. Equivalent to:
     * <pre><code>
        setWaistAngle(waist);
        setShoulderAngle(shoulder);
        setElbowAngle(elbow);
     * </code></pre>
     * @param waist The angle of the waist servo
     * @param shoulder The angle of the shoulder servo
     * @param elbow The angle of the elbow servo
     */
    public void moveTo(double waist, double shoulder, double elbow) {
        setWaistAngle(waist);
        setShoulderAngle(shoulder);
        setElbowAngle(elbow);
    }

    /**
     * Set the target x coordinate of the claw. The unit is 'cubits' (e.g. the length from the elbow
     * servo to the claw).
     * @param value The new x position
     */
    public void setArmX(double value) {
        cx = value;
        updateXY();
    }

    /**
     * Set the target y coordinate of the claw. The unit is 'cubits' (e.g. the length from the elbow
     * servo to the claw).
     * @param value The new y position
     */
    public void setArmY(double value) {
        cy = value;
        updateXY();
    }

    private void updateXY() {
        double dx = cx - x;
        double dy = cy - y;
        double angle1 = atan2(dy,dx);
        double tx = cx - cos(angle1);
        double ty = cy - sin(angle1);
        dx = tx - x2;
        dy = ty - y2;
        double angle2 = atan2(dy, dx);
        setElbowAngle(angle1);
        setShoulderAngle(angle2);
    }
}
