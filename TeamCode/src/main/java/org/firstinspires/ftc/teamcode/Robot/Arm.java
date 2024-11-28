package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@RequiresApi(api = Build.VERSION_CODES.N)
public class Arm {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final DcMotorEx arm;
    private final Servo gripper_rotation_left, gripper_rotation_right;

    private final double LEFT_SAFETY = 1.0-0.08, RIGHT_SAFETY = 0.0+0.08;
    private final double LEFT_AFTERARM = 1.0-0.15, RIGHT_AFTERARM = 0.0+0.15;
    private final double LEFT_INITIAL_POS = 1.0-0.1, RIGHT_INITIAL_POS = 0.0+0.1;
    private final double LEFT_RELEASE_POS = 0.35-0.10, RIGHT_RELEASE_POS = 0.65+0.10;

    Arm(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper_rotation_left = hardwareMap.get(Servo.class, "gripper_rotation_left");
        gripper_rotation_right = hardwareMap.get(Servo.class, "gripper_rotation_right");
    }

    private ScheduledFuture<?> raiseArm = null;

    public void raiseArm(int targetPositionValue, double raisePower) {
        int currentPosition = getCurrentPositionArm();

        arm.setTargetPosition(targetPositionValue);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (currentPosition < targetPositionValue) {
            arm.setPower(raisePower);
        } else{
            arm.setPower(-raisePower);
        }
    }

    public int getCurrentPositionArm() {
        return arm.getCurrentPosition();
    }

    public void gripperInitialPos() {
        gripper_rotation_left.setPosition(LEFT_INITIAL_POS);
        gripper_rotation_right.setPosition(RIGHT_INITIAL_POS);
    }

    public void gripperReleasePos() {
        gripper_rotation_left.setPosition(LEFT_RELEASE_POS);
        gripper_rotation_right.setPosition(RIGHT_RELEASE_POS);
    }

    public void gripperSafety() {
        gripper_rotation_left.setPosition(LEFT_SAFETY);
        gripper_rotation_right.setPosition(RIGHT_SAFETY);
    }
    public void gripperAfterArm() {
        gripper_rotation_left.setPosition(LEFT_AFTERARM);
        gripper_rotation_right.setPosition(RIGHT_AFTERARM);
    }


    public void stopArm() {
        // ----- stopping the slider moving -----
        arm.setPower(0.0);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
    }

}