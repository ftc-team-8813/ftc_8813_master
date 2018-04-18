package org.firstinspires.ftc.teamcode.teleop.util;

import android.widget.Button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.Arm;
import org.firstinspires.ftc.teamcode.autonomous.util.sensors.IMU;
import org.firstinspires.ftc.teamcode.autonomous.util.telemetry.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

import static java.lang.Math.PI;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
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
    //Current arm position
    private double i, j, k, w;
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

    public IronSightsJoystickControl(Gamepad gamepad1, Gamepad gamepad2, Arm arm, Config conf,
                                     Telemetry t, DcMotor base, DcMotor extend, IMU imu, int quadrant) {
        telemetry = t;
        this.base = base;
        this.extend = extend;
        this.imu = imu;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        buttons1 = new ButtonHelper(gamepad1);
        buttons2 = new ButtonHelper(gamepad2);
        driver = new IronSightsArmDriver(arm, conf);
        this.arm = arm;
        this.conf = conf;
        finesse_gain = conf.getDouble("finesse_gain", 0.5);
        home = conf.getDoubleArray("ironSights_home");
        if (home == null) home = new double[] {0, -2, 5, PI};
        out = conf.getDoubleArray("ironSights_out");
        if (out == null) out = new double[] {0, 9, 20, PI};
        i = home[0];
        j = home[1];
        k = home[2];
        w = home[3];
        yaw = conf.getDouble("yaw_mid", 0.5);
        driver.moveArmTo(i, j, k, w);
        arm.moveYaw(yaw);
        arm.openClaw();
    }

    public void start() {
        i = out[0];
        j = out[1];
        k = out[2];
        w = out[3];
        driver.moveArmTo(i, j, k, w);
    }

    public void loop() {
        double dx = -gamepad1.right_stick_x;
        double dy = gamepad1.left_trigger - gamepad1.right_trigger;
        double dz = -gamepad1.right_stick_y;
        double dw = -gamepad1.left_stick_y / 10;
        double dyw = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
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
        double di = dx * cos(base_angle) + dz * sin(base_angle);
        double dj = dy;
        double dk = dx * sin(base_angle) + dz * cos(base_angle);
        i += di;
        j += dj;
        k += dk;
        w += dw;
        yaw += dyw / 50;

        if (w > 3*PI/2) {
            w = 3*PI/2;
        } else if (w < PI/2) {
            w = PI/2;
        }
        if (yaw > 1) {
            yaw = 1;
        } else if (yaw < 0) {
            yaw = 0;
        }
        driver.moveArmTo(i, j, k, w);
        arm.moveYaw(yaw);
        if (buttons1.pressing(ButtonHelper.x)) {
            if (arm.isClawClosed()) {
                arm.openClaw();
            } else {
                arm.closeClaw();
            }
        }

        if (gamepad2.dpad_left) {
            base.setPower(0.8);
        } else if (gamepad2.dpad_right){
            base.setPower(-0.8);
        } else {
            base.setPower(0);
        }

        telemetry.addData("i", i);
        telemetry.addData("j", j);
        telemetry.addData("k", k);
        telemetry.addData("w", w);
        telemetry.addData("Waist", driver.getWaistAngle());
        telemetry.addData("Shoulder", driver.getShoulderAngle());
        telemetry.addData("Elbow", driver.getElbowAngle());
        telemetry.addData("Wrist", driver.getWristAngle());
        telemetry.addData("Turntable Heading", base_angle);
    }

    public void stop() {
        imu.stop();
    }
}
