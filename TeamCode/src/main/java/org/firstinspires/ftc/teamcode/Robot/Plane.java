package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Plane {

    private final double RELEASE_POSITION = 0.3;
    private final double GRAB_POSITION = 0.6;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo plane;

    Plane(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

        plane = hardwareMap.get(Servo.class, "plane_servo");
    }

    public void releasePlane() {
        // rightBumperOnce()
        plane.setPosition(RELEASE_POSITION);
    }

    public void grabPlane() {
        // leftBumberOnce()
        plane.setPosition(GRAB_POSITION);
    }


    public void setGrabPosition() {
        plane.setPosition(GRAB_POSITION);
    }

    public void getCurrentPlaneServoPosition() {
        telemetry.addData("Plane servo position", plane.getPosition());
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}
