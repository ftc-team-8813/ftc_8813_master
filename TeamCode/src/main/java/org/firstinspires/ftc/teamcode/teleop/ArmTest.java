package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptI2cAddressChange;
import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.util.Config;
import org.firstinspires.ftc.teamcode.util.Utils;

/**
 * ArmTest - TeleOp arm drive. Controls:
 * Left stick:
 *   Vertical: Shoulder Y
 *   Horizontal: Base
 * Right stick:
 *   Vertical: Elbow
 *   Horizontal: Shoulder X
 * Right Bumper: Stop
 * A: Open/Close Claw
 * D-Pad:
 *   Vertical: Extend slide
 *   Horizontal: N/A
 *
 */
@TeleOp(name="Arm Test")
public class ArmTest extends OpMode{

    private Servo shoulderX, shoulderY, elbow, claw;
    private DcMotor base, extend;
    private TouchSensor extendLimit;
    private double turnX, turnY, turnElbow;
    private boolean claw_closed = false;
    private double maxTurn;
    private double clawCloseAmount;
    private double clawOpenAmount;
    private Integer extMin = null;
    private int extRange;
    private boolean aHeld = false;
    private Config config = new Config(Config.configFile);

    @Override
    public void init() {
        //Loadsa servos
        shoulderX   = hardwareMap.servo.get("s0"); //Assuming that s0, s1, s2, etc. are names of servos
        shoulderY   = hardwareMap.servo.get("s1");
        elbow       = hardwareMap.servo.get("s2");
        claw        = hardwareMap.servo.get("s3");
        //2 motors
        base        = hardwareMap.dcMotor.get("base");
        extend      = hardwareMap.dcMotor.get("extend");
        //1 bumper
        extendLimit = hardwareMap.touchSensor.get("ext_bumper");

        maxTurn = config.getDouble("servo_turn", 0);
        clawCloseAmount = config.getDouble("claw_closed", 0);
        clawOpenAmount = config.getDouble("claw_open", 0);
        extRange = config.getInt("ext_range", 0);
        if (config.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);

        claw.setPosition(clawOpenAmount);
    }

    @Override
    public void loop() {
        //Assign values
        turnX     +=  gamepad1.right_stick_x * maxTurn;
        turnY     += -gamepad1.left_stick_y * maxTurn;
        turnElbow += -gamepad1.right_stick_y * maxTurn;
        base.setPower(gamepad1.left_stick_x * 0.5);
        if (!extendLimit.isPressed()) {
            //Only allows user to go forward if the minimum switch has been triggered.
            if (gamepad1.dpad_up && extMin != null && extend.getCurrentPosition() < extMin + extRange) {
                extend.setPower(1);
            } else if (gamepad1.dpad_down) {
                extend.setPower(-1);
            } else {
                extend.setPower(0);
            }
        } else {
            extend.setPower(0);
            extMin = extend.getCurrentPosition();
        }
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {}

        //Constrain the values
        turnX = Utils.constrain(turnX, 0, 1);
        turnY = Utils.constrain(turnY, 0, 1);
        turnElbow = Utils.constrain(turnElbow, 0, 1);

        //Set the positions
        shoulderX.setPosition(turnX);
        shoulderY.setPosition(turnY);
        elbow.setPosition(turnElbow);

        if (gamepad1.a) {
            if (!aHeld) {
                aHeld = true;
                claw_closed = !claw_closed;
                if (claw_closed)
                    claw.setPosition(clawCloseAmount);
                else
                    claw.setPosition(0);
            }
        } else {
            aHeld = false;
        }

        String fmt = "%.4f";

        telemetry.addData("X Turn", String.format(fmt, turnX));
        telemetry.addData("Y Turn", String.format(fmt, turnY));
        telemetry.addData("Elbow Turn", String.format(fmt, turnElbow));
        telemetry.addData("Claw Closed", claw_closed);
        telemetry.addData("Extend motor encoder", extend.getCurrentPosition());
        telemetry.addData("Rotation motor encoder", base.getCurrentPosition());
        telemetry.update();
    }
}
