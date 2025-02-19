package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Gripper {
    private final double GRAB_POSITION_1 = 1.0, GRAB_POSITION_2 = +1.0 ;//trebuia e cu smecheria cu axu
    private final double RELEASE_POSITION_1 = GRAB_POSITION_1-1.0;
    private final double RELEASE_POSITION_2 = GRAB_POSITION_2+1.0;

    private final double GRAB_POSITION_3 = 0.0;
    private final double RELEASE_POSITION_3 = -1.0 ;

    /*private final double LEFT_PICKUP = 0.0, RIGHT_PICKUP = 1.0-0.13;
    private final double LEFT_RELEASE = 1.0-0.30, RIGHT_RELEASE = 0.3;
    private final double LEFT_DEFAULT = 0.5, RIGHT_DEFAULT = 0.5;

    private final double OPEN_BARIER_POS = 0.00, CLOSE_BARIER_POS = 0.1;
    */

    private final double PASS_OBJECT_LEFT_PICKUP = 0.85, PASS_OBJECT_LEFT_RELEASE = -0.85;
    private final double PASS_OBJECT_RIGHT_PICKUP = -1, PASS_OBJECT_RIGHT_RELEASE = 0.85;

    private final double SCORE_OBJECT_LEFT_PICKUP = -1, SCORE_OBJECT_LEFT_RELEASE = 1;
    private final double SCORE_OBJECT_RIGHT_PICKUP = 1, SCORE_OBJECT_RIGHT_RELEASE = -1;

    private final double INTAKE_GRIPPER_GRAB = -1, INTAKE_GRIPPER_RELEASE = 1;
    private final double OUTTAKE_GRIPPER_GRAB = -0.3, OUTTAKE_GRIPPER_RELEASE = 0.5;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo leftGripper, rightGripper, intake_gripper;
    private final Servo leftRaiser, rightRaiser, outtake_gripper;

    Gripper(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

        leftGripper = hardwareMap.get(Servo.class, "gripperLeftIntake");
        rightGripper = hardwareMap.get(Servo.class, "gripperRightIntake");

        leftRaiser = hardwareMap.get(Servo.class, "gripperLeftOuttake");
        rightRaiser = hardwareMap.get(Servo.class, "gripperRightOuttake");

        intake_gripper = hardwareMap.get(Servo.class, "gripperIntake");
        outtake_gripper = hardwareMap.get(Servo.class, "gripperOuttake");
    }


    public void intake_grab_position() {
        intake_gripper.setPosition(INTAKE_GRIPPER_GRAB);
    }

    public void intake_release_position() {
        intake_gripper.setPosition(INTAKE_GRIPPER_RELEASE);
    }
    public void intake_release_position_initial() {
        intake_gripper.setPosition(INTAKE_GRIPPER_RELEASE-0.5);
    }

    public void outtake_grab_position() {
        outtake_gripper.setPosition(OUTTAKE_GRIPPER_GRAB);
    }

    public void outtake_release_position() {
        outtake_gripper.setPosition(OUTTAKE_GRIPPER_RELEASE);
    }


    public void pass_object_pickup_position() {
        leftGripper.setPosition(PASS_OBJECT_LEFT_PICKUP);
        rightGripper.setPosition(PASS_OBJECT_RIGHT_PICKUP);
    }

    public void pass_object_release_position() {
        leftGripper.setPosition(PASS_OBJECT_LEFT_RELEASE);
        rightGripper.setPosition(PASS_OBJECT_RIGHT_RELEASE);
    }

    public void score_object_pickup_position() {
        leftRaiser.setPosition(SCORE_OBJECT_LEFT_PICKUP);
        rightRaiser.setPosition(SCORE_OBJECT_RIGHT_PICKUP);
    }

    public void score_object_release_position() {
        leftRaiser.setPosition(SCORE_OBJECT_LEFT_RELEASE);
        rightRaiser.setPosition(SCORE_OBJECT_RIGHT_RELEASE);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}