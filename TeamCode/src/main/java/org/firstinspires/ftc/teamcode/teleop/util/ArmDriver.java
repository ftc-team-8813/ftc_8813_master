package org.firstinspires.ftc.teamcode.teleop.util;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * ArmDriver - Drive the arm servos based on controller input.
 */

public class ArmDriver {

    // Constants -- should be migrated to config?

    public static final double SERVO_MAX_ANGLE = 7.04614352305139340626;

    // Sub-classes

    /**
     * ArmPos - Describes the position of the arm
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

    private final Servo ws, ss, es;
    private double x, y, x2, y2;
    private double cx, cy;
    private double[] currentPositions = new double[3];

    // Constructors

    public ArmDriver(Servo waist, Servo shoulder, Servo elbow) {
        ws = waist;
        ss = shoulder;
        es = elbow;
    }

    // Methods

    // Direct drive
    public void setWaistAngle(double radians) {
        currentPositions[0] = getServoPosition(radians);
    }

    public void setShoulderAngle(double radians) {
        currentPositions[1] = getServoPosition(radians);
    }

    public void setElbowAngle(double radians) {
        currentPositions[2] = getServoPosition(radians);
    }

    public void setArmX(double value) {
        cx = value;
        updateXY();
    }

    public void setArmY(double value) {
        cy = value;
        updateXY();
    }

    private void updateXY() {
        double dx = cx - x;
        double dy = cy - y;
        double angle1 = Math.atan2(dy,dx);
        double tx = cx - Math.cos(angle1);
        double ty = cy - Math.sin(angle1);
        dx = tx - x2;
        dy = ty - y2;
        double angle2 = Math.atan2(dy, dx);
        setElbowAngle(angle1);
        setShoulderAngle(angle2);
    }

    // Static methods

    private static double getServoPosition(double radians) {
        return radians/SERVO_MAX_ANGLE;
    }

    private static double getAngle(double servoPosition) {
        return servoPosition*SERVO_MAX_ANGLE;
    }
}
