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
    public static final double SERVO_ANGLE_V_E = 0.7207;
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
    //Variables
    private double l1, l2;
    private double n;
    private double adj;
    private double theta1, theta2;
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
    public ArmDriver(Servo waist, Servo shoulder, Servo elbow, double l1, double l2) {
        this.l1 = l1;
        this.l2 = l2;
        ws = waist;
        ss = shoulder;
        es = elbow;
        n = l1 + l2;
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
        ws.setPosition(currentPositions[0]);
    }

    /**
     * Set the angle of the shoulder servo in radians
     * @param radians The new angle to target
     */
    private void setShoulderAngle(double radians) {
        currentPositions[1] =
                Utils.scaleRange(radians, 0, PI/2, SERVO_ANGLE_H_S, SERVO_ANGLE_V_S);
        currentPositionsR[1] = radians;
        ss.setPosition(currentPositions[1]);
    }

    /**
     * Set the angle of the elbow servo in radians
     * @param radians The new angle to target
     */
    private void setElbowAngle(double radians) {
        currentPositions[2] =
                Utils.scaleRange(radians, 0, PI/2, SERVO_ANGLE_H_E, SERVO_ANGLE_V_E);
        currentPositionsR[2] = radians;
        es.setPosition(currentPositions[2]);
    }

    //Getters

    public double getClawDistance() {
        return n;
    }

    public double getArmAngle() {
        return adj;
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


    public void moveTo(double distance, double adjust) {
        n = Utils.constrain(distance, 0, l1+l2);
        adj = adjust;
        setShoulderAngle(acos(Utils.constrain(n/(2*l1), 0, 1))+adj);
        setElbowAngle(-2*acos(Utils.constrain(n/(2*l2), 0, 1)));
    }
}
