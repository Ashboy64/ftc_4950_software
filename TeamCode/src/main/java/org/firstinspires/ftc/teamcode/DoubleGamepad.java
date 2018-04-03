package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A helper class that aggregates input from both Gamepads, allowing the
 * drivers to have the same set of controls on both Gamepads and also providing
 * redundancy in case of Gamepad failure during a match. Button presses on
 * either gamepad will be registered, while joystick and trigger input on
 * the first Gamepad will override any input from the second.
 */
public class DoubleGamepad {
    //the two Gamepads used for input
    private final Gamepad GAMEPAD1;
    private final Gamepad GAMEPAD2;

    /**
     * Creates a new DoubleGamepad from the two given gamepads.
     * @param gamepad1 the primary gamepad
     * @param gamepad2 the secondary gamepad
     */
    public DoubleGamepad(Gamepad gamepad1, Gamepad gamepad2) {
        GAMEPAD1 = gamepad1;
        GAMEPAD2 = gamepad2;
    }

    public float leftStickX() {
        return priority(GAMEPAD1.left_stick_x, GAMEPAD2.left_stick_x);
    }

    public float rightStickX() {
        return priority(GAMEPAD1.right_stick_x, GAMEPAD2.right_stick_x);
    }

    public float leftStickY() {
        return priority(GAMEPAD1.left_stick_y, GAMEPAD2.left_stick_y);
    }

    public float rightStickY() {
        return priority(GAMEPAD1.right_stick_y, GAMEPAD2.right_stick_y);
    }

    public boolean leftStickButton() {
        return GAMEPAD1.left_stick_button || GAMEPAD2.left_stick_button;
    }

    public boolean rightStickButton() {
        return GAMEPAD1.right_stick_button || GAMEPAD2.right_stick_button;
    }

    public float leftTrigger() {
        return priority(GAMEPAD1.left_trigger, GAMEPAD2.left_trigger);
    }

    public float rightTrigger() {
        return priority(GAMEPAD1.right_trigger, GAMEPAD2.right_trigger);
    }

    public boolean leftBumper() {
        return GAMEPAD1.left_bumper || GAMEPAD2.left_bumper;
    }

    public boolean rightBumper() {
        return GAMEPAD1.right_bumper || GAMEPAD2.right_bumper;
    }

    public boolean a() {
        return GAMEPAD1.a || GAMEPAD2.a;
    }

    public boolean b() {
        return GAMEPAD1.b || GAMEPAD2.b;
    }

    public boolean x() {
        return GAMEPAD1.x || GAMEPAD2.x;
    }

    public boolean y() {
        return GAMEPAD1.y || GAMEPAD2.y;
    }

    public boolean dpadUp() {
        return GAMEPAD1.dpad_up || GAMEPAD2.dpad_up;
    }

    public boolean dpadDown() {
        return GAMEPAD1.dpad_down || GAMEPAD2.dpad_down;
    }

    public boolean dpadLeft() {
        return GAMEPAD1.dpad_left || GAMEPAD2.dpad_left;
    }

    public boolean dpadRight() {
        return GAMEPAD1.dpad_right || GAMEPAD2.dpad_right;
    }

    public boolean guide() {
        return GAMEPAD1.guide || GAMEPAD2.guide;
    }

    public boolean start() {
        return GAMEPAD1.start || GAMEPAD2.start;
    }

    public boolean back() {
        return GAMEPAD1.back || GAMEPAD2.back;
    }

    //input from the first Gamepad overrides input from the second
    private float priority(float from1, float from2) {
        return from1 == 0 ? from2 : from1;
    }
}