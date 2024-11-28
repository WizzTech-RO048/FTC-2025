package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
    private Gamepad gamepad;

    private int x, y, a, b;
    private int dpad_up, dpad_down, dpad_right, dpad_left;
    private int left_bumper, right_bumper;
    private int start_button;

    public double left_stick_x, left_stick_y;
    public double right_stick_x, right_stick_y;

    public double left_trigger, right_trigger;

    public Controller(Gamepad g) {
        gamepad = g;
    }

    public void update() {
        // ------ reading data from the controller buttons ------
        if(gamepad.x) { x++; } else { x = 0; }
        if(gamepad.y) { y++; } else { y = 0; }
        if(gamepad.a) { a++; } else { a = 0; }
        if(gamepad.b) { b++; } else { b = 0; }

        // ----- reading data from the dpad buttons -----
        if(gamepad.dpad_up) { dpad_up++; } else { dpad_up = 0; }
        if(gamepad.dpad_down) { dpad_down++; } else { dpad_down = 0; }
        if(gamepad.dpad_right) { dpad_right++; } else { dpad_right = 0; }
        if(gamepad.dpad_left) { dpad_left++; } else { dpad_left = 0; }

        // -------- reading data from bumpers and triggers -------
        if(gamepad.left_bumper) { left_bumper++; } else { left_bumper = 0; }
        if(gamepad.right_bumper) { right_bumper++; } else { right_bumper = 0; }

        if(gamepad.start) { start_button++; } else { start_button = 0; }

        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;

        // -------- reading data from the joysticks ------
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
    }

    public boolean dpadUp() { return dpad_up > 0; }
    public boolean dpadDown() { return dpad_down > 0; }
    public boolean dpadLeft() { return dpad_left > 0; }
    public boolean dpadRight() { return dpad_right > 0; }

    public boolean dpadUpOnce() { return dpad_up == 1; }
    public boolean dpadDownOnce() { return dpad_down == 1; }
    public boolean dpadLeftOnce() { return dpad_left == 1; }
    public boolean dpadRightOnce() { return dpad_right == 1; }

    public boolean X() { return x > 0; }
    public boolean Y() { return y > 0; }
    public boolean A() { return a > 0; }
    public boolean B() { return b > 0; }

    public boolean XOnce() { return x == 1; }
    public boolean YOnce() { return y == 1; }
    public boolean AOnce() { return a == 1; }
    public boolean BOnce() { return b == 1; }

    public boolean leftBumper() { return left_bumper > 0; }
    public boolean rightBumper() { return right_bumper > 0; }

    public boolean leftBumperOnce() { return left_bumper == 1; }
    public boolean rightBumperOnce() { return right_bumper == 1; }

    public boolean startButtonOnce() { return start_button == 1; }

}