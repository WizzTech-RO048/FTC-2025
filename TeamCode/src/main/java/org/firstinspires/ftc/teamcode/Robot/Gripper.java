package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Gripper {
    private final double GRAB_POSITION_1 = 1.0, GRAB_POSITION_2 = +1.0 ;//trebuie cu smecheria cu axu
    private final double RELEASE_POSITION_1 = GRAB_POSITION_1-1.0;
    private final double RELEASE_POSITION_2 = GRAB_POSITION_2+1.0;

    private final double GRAB_POSITION_3 = 0.0;
    private final double RELEASE_POSITION_3 = -1.0 ;

    /*private final double LEFT_PICKUP = 0.0, RIGHT_PICKUP = 1.0-0.13;
    private final double LEFT_RELEASE = 1.0-0.30, RIGHT_RELEASE = 0.3;
    private final double LEFT_DEFAULT = 0.5, RIGHT_DEFAULT = 0.5;

    private final double OPEN_BARIER_POS = 0.00, CLOSE_BARIER_POS = 0.1;
    */
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

    private final Servo claw;
    private final Servo left_turner, right_turner;



    Gripper(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

        claw = hardwareMap.get(Servo.class, "claw");
        left_turner = hardwareMap.get(Servo.class, "left_turner");
        right_turner = hardwareMap.get(Servo.class, "right_turner");
    }


    public void grab_position() {
        left_turner.setPosition(0.4);
        right_turner.setPosition(0.5);
    }

    public void release_position() {
        left_turner.setPosition(0.5);//daca nu mai mare atunci
        right_turner.setPosition(-0);
    }

    public void no_position(){
    }

    public void grab()
    {
        claw.setPosition(GRAB_POSITION_3);
    }

    public void release()
    {
        claw.setPosition(RELEASE_POSITION_3);
    }
    public double getposgripper()
    {
        return claw.getPosition();
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}