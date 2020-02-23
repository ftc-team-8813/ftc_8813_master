package org.firstinspires.ftc.teamcode.autonomous.util.opmodes;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.util.Config;
import org.firstinspires.ftc.teamcode.common.util.DataStorage;
import org.firstinspires.ftc.teamcode.common.util.TelemetryWrapper;
import org.firstinspires.ftc.teamcode.common.util.concurrent.GlobalThreadPool;
import org.firstinspires.ftc.teamcode.teleop.BaseTeleOp;
import org.firstinspires.ftc.teamcode.teleop.util.ButtonHelper;

import java.io.File;
import java.util.Arrays;
import java.util.concurrent.Future;

@TeleOp(name="Autonomous Creator")
public class AutoDesigner extends BaseTeleOp
{
    private static final int STATE_INIT = 0;
    private static final int STATE_CHOOSE_OP = 1;
    private static final int STATE_CREATE_OP = 2;
    private static final int STATE_RUN_OP = 3;
    private static final int STATE_REVERT_OP = 4;
    private static final int STATE_EDIT_OP = 5;
    
    public static final int OP_FORWARD = 0; // left joystick y
    public static final int OP_RIGHT = 1;   // left joystick x
    public static final int OP_TURN = 2;    // right joystick y
    public static final int OP_HOOK = 3;    // y
    public static final int OP_INTAKE = 4;  // left/right bumper
    public static final int OP_LIFT = 5;    // left/right trigger -> left joystick y
    public static final int OP_ARM = 6;     // dpad up/down -> left joystick y
    public static final int OP_CLAW = 7;    // dpad left
    
    private static final String[] ops = {"Forward", "Strafe", "Turn", "Hook", "Intake", "Lift", "Claw", "Claw"};
    
    private DataStorage data;
    private ButtonHelper buttons;
    private int state;
    private int op;
    private int op_index;
    private int op_count;
    private int[] params;
    
    private boolean running;
    private Future<?> daemon;
    
    private Robot robot;
    
    @Override
    public void init()
    {
        super.init();
        TelemetryWrapper.init(telemetry, 10);
        data = new DataStorage(new File(Config.storageDir + "autonomous.json"));
        state = STATE_INIT;
        op = -1;
        buttons = new ButtonHelper(gamepad1);
        robot = Robot.instance();
        
        /*
        ((AccelMotor)robot.drivetrain.leftFront.getMotor()).setDefaultAcceleration();
        ((AccelMotor)robot.drivetrain.rightFront.getMotor()).setDefaultAcceleration();
        ((AccelMotor)robot.drivetrain.leftBack.getMotor()).setDefaultAcceleration();
        ((AccelMotor)robot.drivetrain.rightBack.getMotor()).setDefaultAcceleration();
         */
        
        robot.imu.setImmediateStart(true);
        robot.imu.initialize();
    }
    
    @Override
    public void start()
    {
        // TODO: File chooser
    }
    
    @Override
    public void doLoop()
    {
        switch (state)
        {
            case STATE_INIT:
            {
                if (data.getInt("op_count", 0) == 0)
                {
                    state = STATE_CHOOSE_OP;
                    break;
                }
                op_count = data.getInt("op_count", 0);
                TelemetryWrapper.setLine(0, "Program file found!");
                TelemetryWrapper.setLine(1, "Press B to edit it");
                TelemetryWrapper.setLine(2, "Press LEFT BUMPER to clear it");
                if (buttons.pressing(ButtonHelper.b))
                {
                    fetchOp();
                    state = STATE_EDIT_OP; // Index = 0
                }
                else if (buttons.pressing(ButtonHelper.left_bumper))
                {
                    state = STATE_CHOOSE_OP;
                    data.clear();
                    op_count = 0;
                }
                break;
            }
            case STATE_CHOOSE_OP:
            {
                TelemetryWrapper.setLine(0, "Choose an operation by moving the appropriate control");
                TelemetryWrapper.setLine(1, "Press X to continue");
                String opname = (op < 0 ? "Nothing" : ops[op]);
                TelemetryWrapper.setLine(2, "Selected move: " + opname);
                if (op_index > 0)
                    TelemetryWrapper.setLine(3, "Press BACK to go back one step");
                else
                    TelemetryWrapper.setLine(3, "");
                boolean automatic = false;
                if (Math.abs(gamepad1.left_stick_y) > 0.8)
                {
                    op = OP_FORWARD;
                }
                else if (Math.abs(gamepad1.left_stick_x) > 0.8)
                {
                    op = OP_RIGHT;
                }
                else if (Math.abs(gamepad1.right_stick_y) > 0.8)
                {
                    op = OP_TURN;
                }
                else if (buttons.pressing(ButtonHelper.b))
                {
                    op = OP_HOOK;
                    automatic = true;
                }
                else if (buttons.pressing(ButtonHelper.left_bumper) || buttons.pressing(ButtonHelper.right_bumper))
                {
                    op = OP_INTAKE;
                    automatic = true;
                }
                else if (gamepad1.left_trigger > 0.3 || gamepad1.right_trigger > 0.3)
                {
                    op = OP_LIFT;
                }
                else if (gamepad1.dpad_up || gamepad1.dpad_down)
                {
                    op = OP_ARM;
                }
                else if (gamepad1.dpad_left)
                {
                    op = OP_CLAW;
                    automatic = true;
                }
                
                if ((automatic || buttons.pressing(ButtonHelper.x)) && op >= 0)
                {
                    initCreateOp();
                    state = STATE_CREATE_OP;
                }
                else if (buttons.pressing(ButtonHelper.back))
                {
                    state = STATE_REVERT_OP;
                }
                break;
            }
            case STATE_CREATE_OP:
            {
                TelemetryWrapper.setLine(0, "Press B when complete");
                TelemetryWrapper.setLine(1, "");
                TelemetryWrapper.setLine(2, "");
                TelemetryWrapper.setLine(3, "");
                boolean finished = false;
                
                if (op == OP_FORWARD)
                {
                    robot.drivetrain.drive(-gamepad1.left_stick_y * 0.3, 0, 0);
                }
                else if (op == OP_RIGHT)
                {
                    robot.drivetrain.drive(0, gamepad1.left_stick_x * 0.3, 0);
                }
                else if (op == OP_TURN)
                {
                    robot.drivetrain.drive(0, 0, -gamepad1.right_stick_y * 0.3);
                }
                else if (op == OP_HOOK)
                {
                    robot.foundationhook.toggle();
                    finished = true;
                }
                else if (op == OP_INTAKE)
                {
                    robot.intake.collectStone(params[0]/100.0);
                }
                else if (op == OP_LIFT)
                {
                    robot.slide.oldRaiseLift(-gamepad1.left_stick_y * 0.3);
                }
                else if (op == OP_ARM)
                {
                    robot.newarm.moveArm(-gamepad1.left_stick_y * 0.3);
                }
                else if (op == OP_CLAW)
                {
                    robot.claw.toggleClaw();
                    finished = true;
                }
                
                if (buttons.pressing(ButtonHelper.b) || finished)
                {
                    robot.drivetrain.drive(0, 0, 0);
                    finishCreateOp();
                    op_index++;
                    if (op_index == op_count)
                    {
                        state = STATE_CHOOSE_OP;
                    }
                    else
                    {
                        fetchOp();
                        state = STATE_EDIT_OP;
                    }
                }
                break;
            }
            case STATE_EDIT_OP:
            {
                TelemetryWrapper.setLine(0, "Press START to execute the selected operation");
                TelemetryWrapper.setLine(1, "Press BACK to go back");
                TelemetryWrapper.setLine(2, "Press B to edit the selected operation");
                TelemetryWrapper.setLine(3, "Selected op: " + showOp());
                
                if (buttons.pressing(ButtonHelper.start))
                {
                    state = STATE_RUN_OP;
                }
                else if (buttons.pressing(ButtonHelper.back))
                {
                    state = STATE_REVERT_OP;
                }
                else if (buttons.pressing(ButtonHelper.b))
                {
                    state = STATE_CHOOSE_OP;
                }
                break;
            }
            case STATE_RUN_OP:
            {
                TelemetryWrapper.setLine(0, "Running... please wait");
                TelemetryWrapper.setLine(1, "");
                TelemetryWrapper.setLine(2, "");
                TelemetryWrapper.setLine(3, "");
                if (!running)
                {
                    running = true;
                    daemon = GlobalThreadPool.instance().start(() -> executeOp(1));
                }
                else if (daemon.isDone())
                {
                    running = false;
                    op_index += 1;
                    if (op_index == op_count)
                    {
                        state = STATE_CHOOSE_OP;
                    }
                    else
                    {
                        fetchOp();
                        state = STATE_EDIT_OP;
                    }
                }
                break;
            }
            case STATE_REVERT_OP:
            {
                TelemetryWrapper.setLine(0, "Running... please wait");
                TelemetryWrapper.setLine(1, "");
                TelemetryWrapper.setLine(2, "");
                TelemetryWrapper.setLine(3, "");
                if (!running)
                {
                    op_index -= 1;
                    if (op_index < 0)
                    {
                        op_index = 0;
                        if (op_count == 0)
                        {
                            state = STATE_CHOOSE_OP;
                        }
                        else
                        {
                            fetchOp();
                            state = STATE_EDIT_OP;
                        }
                        break;
                    }
                    
                    fetchOp();
                    running = true;
                    daemon = GlobalThreadPool.instance().start(() -> executeOp(-1));
                }
                else if (daemon.isDone())
                {
                    running = false;
                    state = STATE_EDIT_OP;
                }
                break;
            }
        }
        TelemetryWrapper.setLine(4, "----DEBUG----");
        TelemetryWrapper.setLine(5, "Op index: " + op_index);
        TelemetryWrapper.setLine(6, "Params: " + Arrays.toString(params));
        TelemetryWrapper.setLine(7, "State: " + state);
        TelemetryWrapper.setLine(8, "Op count: " + op_count);
        TelemetryWrapper.setLine(9, "Motor positions: "
                                            + robot.drivetrain.leftFront.getCurrentPosition() + " "
                                            + robot.drivetrain.rightFront.getCurrentPosition() + " "
                                            + robot.drivetrain.leftBack.getCurrentPosition() + " "
                                            + robot.drivetrain.rightBack.getCurrentPosition());
    }
    
    private void initCreateOp()
    {
        if (op == OP_FORWARD || op == OP_RIGHT)
        {
            params = new int[1];
            params[0] = robot.drivetrain.rightBack.getCurrentPosition();
        }
        else if (op == OP_TURN)
        {
            params = new int[1];
            params[0] = (int)robot.imu.getHeading();
        }
        else if (op == OP_HOOK)
        {
            params = new int[1];
        }
        else if (op == OP_INTAKE)
        {
            params = new int[2];
            params[0] = (int)System.currentTimeMillis();
            if (gamepad1.left_bumper) params[1] = -20;
            else if (gamepad1.right_bumper) params[1] = 40;
        }
        else if (op == OP_LIFT)
        {
            params = new int[2];
            params[1] = (int)robot.slide.getCurrentPos();
        }
        else if (op == OP_ARM)
        {
            params = new int[2];
            params[1] = robot.newarm.motorArm.getCurrentPosition();
        }
        else if (op == OP_CLAW)
        {
            params = new int[1];
        }
    }
    
    private void finishCreateOp()
    {
        if (op == OP_FORWARD)
        {
            params[0] = robot.drivetrain.rightBack.getCurrentPosition() - params[0];
        }
        else if (op == OP_RIGHT)
        {
            params[0] = robot.drivetrain.rightBack.getCurrentPosition() - params[0];
        }
        else if (op == OP_TURN)
        {
            params[0] = (int)robot.imu.getHeading() - params[0];
        }
        else if (op == OP_HOOK)
        {
            params[0] = robot.foundationhook.hookDown() ? 1 : 0;
        }
        else if (op == OP_INTAKE)
        {
            params[0] = (int)System.currentTimeMillis() - params[0];
            robot.intake.stopIntake();
        }
        else if (op == OP_LIFT)
        {
            params[0] = (int)robot.slide.getCurrentPos();
        }
        else if (op == OP_ARM)
        {
            params[0] = robot.newarm.motorArm.getCurrentPosition();
        }
        else if (op == OP_CLAW)
        {
            params[0] = robot.claw.clawClosed() ? 1 : 0;
        }
        
        JsonObject operation = new JsonObject();
        operation.addProperty("op", op);
        JsonArray paramsJson = new JsonArray();
        for (int i = 0; i < params.length; i++)
        {
            paramsJson.add(params[i]);
        }
        operation.add("params", paramsJson);
        data.addJson(Integer.toString(op_index), operation);
        if (op_index == op_count)
        {
            op_count = op_index + 1;
        }
    }
    
    private void fetchOp()
    {
        JsonObject operation = (JsonObject)data.getJson(Integer.toString(op_index));
        op = operation.get("op").getAsInt();
        JsonArray paramsJson = operation.getAsJsonArray("params");
        params = new int[paramsJson.size()];
        for (int i = 0; i < params.length; i++)
        {
            params[i] = paramsJson.get(i).getAsInt();
        }
    }
    
    private void executeOp(int direction)
    {
        try
        {
            if (op == OP_FORWARD)
            {
                robot.drivetrain.move(0.3, 0, 0, params[0] * direction);
            }
            else if (op == OP_RIGHT)
            {
                robot.drivetrain.move(0, 0.3, 0, params[0] * direction);
            }
            else if (op == OP_TURN)
            {
                robot.drivetrain.move(0, 0, 0.3, params[0] * direction);
            }
            else if (op == OP_HOOK)
            {
                boolean closed = (params[0] == 1);
                if (direction == -1) closed = !closed;
                if (closed) robot.foundationhook.moveHookDown();
                else robot.foundationhook.moveHookUp();
            }
            else if (op == OP_INTAKE)
            {
                robot.intake.collectStone(params[1]/100.0 * direction);
                Thread.sleep(params[0]);
                robot.intake.stopIntake();
            }
            else if (op == OP_LIFT)
            {
                int pos = params[0];
                if (direction == -1) pos = params[1];
                robot.slide.raiseLiftEnc(1, pos);
            }
            else if (op == OP_ARM)
            {
                int pos = params[0];
                if (direction == -1) pos = params[1];
                robot.newarm.moveArmTo(0.4, pos);
            }
            else if (op == OP_CLAW)
            {
                boolean closed = (params[0] == 1);
                if (direction == -1) closed = !closed;
                if (closed) robot.claw.closeClaw();
                else robot.claw.openClaw();
            }
        } catch (InterruptedException e) {
            log.w("Interrupted while running");
        }
    }
    
    private String showOp()
    {
        return ops[op] + " " + Arrays.toString(params);
    }
    
    @Override
    public void stop()
    {
        super.stop();
        data.addNumber("op_count", op_count);
        data.save();
    }
    
}
