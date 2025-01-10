package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.RequiresApi;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.ScheduledExecutorService;


@RequiresApi(api = Build.VERSION_CODES.N)
public class Robot {

    public Telemetry telemetry;

    public Slider slider;
    public Arm arm;

    public Gripper gripper;
    public Lift lift;

    public Robot(final HardwareMap hardwareMap, final Telemetry t, ScheduledExecutorService scheduler) {
        telemetry = t;

        // --------- initializing the imu sensor --------
        // BNO055IMU imu_sensor = hardwareMap.get(BNO055IMU.class, "imu_sensor");
        // imu_sensor.initialize(new BNO055IMU.Parameters());

        Slider.Parameters slider_parameters = new Slider.Parameters();
//        slider_parameters.armRaisedPosition = 6100; // 5200 is the maximum
        slider_parameters.telemetry = telemetry;
        slider_parameters.hardwareMap = hardwareMap;
        slider_parameters.scheduler = scheduler;
        slider = new Slider(slider_parameters);

        Arm.Parameters arm_parameters = new Arm.Parameters();
        arm_parameters.telemetry = telemetry;
        arm_parameters.hardwareMap = hardwareMap;
        arm_parameters.scheduler = scheduler;
        arm = new Arm(arm_parameters);




        Gripper.Parameters gripper_parameters = new Gripper.Parameters();
        gripper_parameters.telemetry = telemetry;
        gripper_parameters.hardwareMap = hardwareMap;
        gripper = new Gripper(gripper_parameters);

        Lift.Parameters lift_parameters = new Lift.Parameters();
        lift_parameters.telemetry = telemetry;
        lift_parameters.hardwareMap = hardwareMap;
        lift_parameters.scheduler = scheduler;
        lift = new Lift(lift_parameters);

    }

    public void getCurrentPos() {
        telemetry.addData("Slider position", slider.getCurrentPositionSlider());
        telemetry.update();
    }

}