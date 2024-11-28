//package org.firstinspires.ftc.teamcode.Robot;
//
//import android.os.Build;
//import android.util.Pair;
//import androidx.annotation.NonNull;
//import androidx.annotation.RequiresApi;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import java.util.Objects;
//import java.util.concurrent.ScheduledExecutorService;
//import java.util.concurrent.ScheduledFuture;
//import java.util.concurrent.TimeUnit;
//
//@RequiresApi(api = Build.VERSION_CODES.N)
//public class Slider {
//
//    private final Telemetry telemetry;
//    private final HardwareMap hardwareMap;
//    private final ScheduledExecutorService scheduler;
//
//    private final DcMotorEx slider;
//
//    private final int leftSliderLimit, rightSliderLimit;
//
//    Slider(@NonNull final Parameters parameters) {
//        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
//        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
//        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");
//
//        slider = hardwareMap.get(DcMotorEx.class, "slider");
//        slider.setDirection(DcMotorSimple.Direction.REVERSE);
//        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftSliderLimit = parameters.leftSliderLimit;
//        rightSliderLimit = parameters.rightSliderLimit;
//    }
//
//    private ScheduledFuture<?> lastSliderMove = null;
//
//    public ScheduledFuture<?> raiseSlider(int targetPositionValue, double raisePower) {
//        if (!Utils.isDone(lastSliderMove) && !lastSliderMove.cancel(true)) {
//            return null;
//        }
//
//        int initialPosition = slider.getCurrentPosition();
//
//
//        if (initialPosition == targetPositionValue) {
//            return null;
//        }
//
//        slider.setTargetPosition(targetPositionValue);
//        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slider.setPower(targetPositionValue > initialPosition ? -raisePower : raisePower);
//
//        lastSliderMove = Utils.poll(scheduler, () -> !slider.isBusy(), () -> slider.setPower(0), 10, TimeUnit.MILLISECONDS);
//
//        return lastSliderMove;
//    }
//
//    public void slidermove() {
//        slider.setPower(1.0);
//    }
//
//    public int getCurrentPositionSlider() {
//        return slider.getCurrentPosition();
//    }
//
//    public static class Parameters {
//        public HardwareMap hardwareMap;
//        public Telemetry telemetry;
//        public ScheduledExecutorService scheduler;
//        public int leftSliderLimit;
//        public int rightSliderLimit;
//    }
//
//}

package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@RequiresApi(api = Build.VERSION_CODES.N)
public class Slider {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final DcMotorEx slider;

//    private final int armRaisedPosition;

    Slider(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private ScheduledFuture<?> raiseSlider = null;

    public void raiseSlider(int targetPositionValue, double raisePower) {
        int currentPosition = getCurrentPositionSlider();

        slider.setTargetPosition(targetPositionValue);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (currentPosition > targetPositionValue) {
            slider.setPower(raisePower);
        } else {
            slider.setPower(-raisePower);
        }
    }

    public int getCurrentPositionSlider() {
        return slider.getCurrentPosition();
    }

    public void stopSlider() {
        // ----- stopping the slider moving -----
        slider.setPower(0.0);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
    }

}