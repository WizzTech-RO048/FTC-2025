package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;

public class HorizontalSlider {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final Servo HorizontalSliderLeft,HorizontalSliderRight;

    private final int leftExtended = -1, leftStationed = 1;
    private final int rightExtended = 1, rightStationed = -1;

    HorizontalSlider(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        HorizontalSliderLeft = hardwareMap.get(Servo.class, "SliderLeft");
        HorizontalSliderRight = hardwareMap.get(Servo.class, "SliderRight");

    }

        public void setExtendedPosition() {
        HorizontalSliderLeft.setPosition(leftExtended);
        HorizontalSliderRight.setPosition(rightExtended);
    }

    public void setStationaryPosition() {
        HorizontalSliderLeft.setPosition(leftStationed);
        HorizontalSliderRight.setPosition(rightStationed);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
        public int leftSliderLimit;
        public int rightSliderLimit;
    }

}
