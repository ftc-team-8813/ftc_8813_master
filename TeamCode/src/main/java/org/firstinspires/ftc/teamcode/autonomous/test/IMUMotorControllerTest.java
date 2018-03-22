package org.firstinspires.ftc.teamcode.autonomous.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.BaseAutonomous;
import org.firstinspires.ftc.teamcode.autonomous.tasks.TaskRotate;
import org.firstinspires.ftc.teamcode.autonomous.util.IMUMotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.MotorController;
import org.firstinspires.ftc.teamcode.autonomous.util.sensors.IMU;

/**
 * Created by aidan on 3/20/18.
 */
@Autonomous(name="IMU Control Test")
public class IMUMotorControllerTest extends BaseAutonomous {
    @Override
    public void run() throws InterruptedException {
        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.initialize(telemetry);
        imu.start();
        MotorController controller = new IMUMotorController(hardwareMap.dcMotor.get("base"), imu);
        tasks.add(new TaskRotate(controller, 0));
        runTasks();
        while (opModeIsActive()) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                break;
            }
        }
        controller.close();
    }
}
