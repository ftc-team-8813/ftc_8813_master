package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskExtendSlide;
import org.firstinspires.ftc.teamcode.autonomous.util.Config;
import org.firstinspires.ftc.teamcode.autonomous.util.arm.RobotMove;
import org.firstinspires.ftc.teamcode.util.Utils;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * MoveRecorder - A duplicate of ArmTest that records movements. Controls:
 * Left stick:
 *   Vertical: Shoulder Y
 *   Horizontal: Base
 * Right stick:
 *   Vertical: Elbow
 *   Horizontal: Shoulder X
 * Right Bumper: Stop
 * A: Open/Close Claw
 * Y: Start/Stop Recording
 * D-Pad:
 *   Vertical: Extend slide
 *   Horizontal: N/A
 *
 */
@TeleOp(name="Autonomous Move Recorder")
public class MoveRecorder extends OpMode{

    /* Servos */
    private Servo shoulderX, shoulderY, elbow, claw;
    /* DC motors */
    private DcMotor base, extend;
    /* Limit switch */
    private TouchSensor extendLimit;
    /* Rotation values */
    private double turnX, turnY, turnElbow;
    private boolean claw_closed = false;
    /* Maximum amount to be added to servo position every 200 ms */
    private double maxTurn;
    /* Claw constants */
    private double clawCloseAmount;
    private double clawOpenAmount;
    /* Extend motor minimum encoder value */
    private Integer extMin = null;
    /* Constant range for extension motor */
    private int extRange;
    /* Stores whether a button is held */
    private boolean aHeld = false;
    private boolean yHeld = false;
    /* Configuration */
    private Config config = new Config(Config.configFile);

    /* Robot movement recording */
    private List<RobotMove> moves = new ArrayList<>();
    private boolean recording = false;
    private long last;
    private long startTime;

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
        extendLimit = new TouchSensor() {
            private DigitalChannel dc = hardwareMap.digitalChannel.get("limit");
            @Override
            public double getValue() {
                return dc.getState() ? 1 : 0;
            }

            @Override
            public boolean isPressed() {
                return getValue() == 1;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }

            @Override
            public String getDeviceName() {
                return null;
            }

            @Override
            public String getConnectionInfo() {
                return null;
            }

            @Override
            public int getVersion() {
                return 0;
            }

            @Override
            public void resetDeviceConfigurationForOpMode() {}

            @Override
            public void close() {}
        };

        maxTurn = config.getDouble("servo_turn", 0);
        clawCloseAmount = config.getDouble("claw_closed", 0);
        clawOpenAmount = config.getDouble("claw_open", 0);
        extRange = config.getInt("ext_range", 0);
        if (config.getBoolean("base_reverse", false))
            base.setDirection(DcMotorSimple.Direction.REVERSE);
        if (config.getBoolean("ext_reverse", false))
            extend.setDirection(DcMotorSimple.Direction.REVERSE);
        if (TaskExtendSlide.extended)
            extMin = 0;


        shoulderX.setPosition(0.4154);
        claw.setPosition(clawOpenAmount);
    }

    @Override
    public void loop() {
        boolean action = true;
        //Assign values
        turnX     +=  gamepad1.right_stick_x * maxTurn;
        turnY     += -gamepad1.left_stick_y * maxTurn;
        turnElbow += -gamepad1.right_stick_y * maxTurn;
        base.setPower(gamepad1.left_stick_x * 0.25);
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
        } catch (InterruptedException e) {
            return;
        }

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
                    claw.setPosition(clawOpenAmount);
            }
        } else {
            aHeld = false;
        }

        if (gamepad1.y) {
            if (!yHeld) {
                yHeld = true;
                recording = !recording;
                if (!recording)
                    save();
                else
                    startTime = System.currentTimeMillis();
            }
        } else {
            yHeld = false;
        }

        if (!recording)
            action = false;

        if (action) {
            RobotMove move = new RobotMove();
            if (last != 0)
                move.dt = (int) (System.currentTimeMillis() - last);
            last = System.currentTimeMillis();
            move.xTurn = turnX;
            move.yTurn = turnY;
            move.clawPos = (claw_closed ? clawCloseAmount : clawOpenAmount);
            move.elbow = turnElbow;
            move.extendPower = extend.getPower();
            move.basePower = base.getPower();
            move.extSet = extMin != null;
            RobotMove last = moves.size() == 0 ? null : moves.get(moves.size()-1);
            if (last != null && last.equals(move)) {
                //Add the time of this one onto the other one
                last.dt += move.dt;
            } else {
                moves.add(move);
            }
        }

        String fmt = "%.4f";

        telemetry.addData("X Turn", String.format(fmt, turnX));
        telemetry.addData("Y Turn", String.format(fmt, turnY));
        telemetry.addData("Elbow Turn", String.format(fmt, turnElbow));
        telemetry.addData("Claw Closed", claw_closed);
        telemetry.addData("Extend motor encoder", extend.getCurrentPosition());
        telemetry.addData("Rotation motor encoder", base.getCurrentPosition());
        telemetry.addData("Recording", recording);
        if (recording)
            telemetry.addData("Elapsed time: ", (System.currentTimeMillis() - startTime)/1000);
        telemetry.update();
    }

    public void stop() {
        save();
    }

    private void save() {
        try {
            File out = new File(
                    Config.storageDir + "/drive_" + Long.toHexString(System.currentTimeMillis()/1000) + ".dat");
            DataOutputStream dat = new DataOutputStream(
                    new FileOutputStream(out));
            dat.writeInt(moves.size());
            for (RobotMove move : moves) {
                move.write(dat);
            }
            dat.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
