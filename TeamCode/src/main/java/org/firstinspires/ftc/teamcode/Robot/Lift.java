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
public class Lift {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final DcMotorEx right_lift, left_lift;

    private final Servo lift_servo_right, lift_servo_left;
    private final double LEFT_LIFT_UP = 1, RIGHT_LIFT_UP = -0.8;
    private final double LEFT_LIFT_DOWN = -0.8  , RIGHT_LIFT_DOWN = +0.6;


    Lift(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        right_lift = hardwareMap.get(DcMotorEx.class, "scul_motor_right");
        right_lift.setDirection(DcMotorSimple.Direction.FORWARD);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_lift = hardwareMap.get(DcMotorEx.class, "scul_motor_left");
        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_servo_left = hardwareMap.get(Servo.class, "servo_left");
        lift_servo_right = hardwareMap.get(Servo.class, "servo_right");
    }

    private ScheduledFuture<?> raiseArmLiftRight = null, raiseArmLiftLeft = null;

    public ScheduledFuture<?> liftUpRight(int targetPositionValue, double raisePower) {
        if (!Utils.isDone(raiseArmLiftRight) && !raiseArmLiftRight.cancel(true)) {
            return null;
        }

//        int targetPosition = (int) Math.floor(Utils.interpolate(0, armRaisedPosition, positionPercentage, 1));
        int initialPosition = right_lift.getCurrentPosition();

        if (targetPositionValue == initialPosition) {
            return null;
        }

        right_lift.setTargetPosition(targetPositionValue);
        right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_lift.setPower(targetPositionValue > initialPosition ? raisePower : -raisePower);

        raiseArmLiftRight = Utils.poll(scheduler, () -> !right_lift.isBusy(), () -> right_lift.setPower(0), 10, TimeUnit.MILLISECONDS);

        return raiseArmLiftRight;
    }

    public ScheduledFuture<?> liftUpLeft(int targetPositionValue, double raisePower) {
        if (!Utils.isDone(raiseArmLiftLeft) && !raiseArmLiftLeft.cancel(true)) {
            return null;
        }

//        int targetPosition = (int) Math.floor(Utils.interpolate(0, armRaisedPosition, positionPercentage, 1));
        int initialPosition = left_lift.getCurrentPosition();

        if (targetPositionValue == initialPosition) {
            return null;
        }

        left_lift.setTargetPosition(targetPositionValue);
        left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift.setPower(targetPositionValue > initialPosition ? raisePower : -raisePower);

        raiseArmLiftLeft = Utils.poll(scheduler, () -> !left_lift.isBusy(), () -> left_lift.setPower(0), 10, TimeUnit.MILLISECONDS);

        return raiseArmLiftLeft;
    }

    public void setUpPosition() {
        lift_servo_left.setPosition(LEFT_LIFT_UP);
        lift_servo_right.setPosition(RIGHT_LIFT_UP);
    }

    public void setDownPosition() {
        lift_servo_left.setPosition(LEFT_LIFT_DOWN);
        lift_servo_right.setPosition(RIGHT_LIFT_DOWN);
    }

    public double getCurrentPositionServoLeft() {
        return lift_servo_left.getPosition();
    }
    public double getCurrentPositionServoRight()
    {
        return lift_servo_right.getPosition();
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
    }

}