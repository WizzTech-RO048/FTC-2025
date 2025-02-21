package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;

public class HorizontalSlider {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo HorizontalSliderLeft, HorizontalSliderRight;

    private final double leftExtended = 1, leftStationed = 0.8;
    private final double rightExtended = -1, rightStationed = -0.9;

    HorizontalSlider(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        HorizontalSliderLeft = hardwareMap.get(Servo.class, "sliderLeft");
        HorizontalSliderRight = hardwareMap.get(Servo.class, "sliderRight");
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
    }
}
