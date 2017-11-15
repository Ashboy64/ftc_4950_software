package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by rao_a on 11/14/2017.
 */

@TeleOp(name="FinalTeleOp", group ="Concept")
public class FinalTele extends OpMode{
    CRServo clampServo; // Servo that controls the clamping motion of the glyph mechanism
    DcMotor armMotor; // Motor used to raise/lower glyph mechanism
    DcMotor motorRight; // Right driving motor
    DcMotor motorLeft; // Left driving motor
    float rt; // Right trigger, raises arm when pressed
    float lt; // Left trigger, lowers arm when pressed
    boolean rb; // Right bumper, opens clamp when pressed
    boolean lb; // Left bumper, closes clamp when pressed
    float rj; // Right joystick, controls right motor
    float lj; // Left joystick, controls left motor
    double pr1 = 0.5; // Power to raise the glyph mechanism while it's position is before the highest
    double pr2 = -0.05; // Power to raise the glyph mechanism while it's position is past the highest
    double pl1 = 0.05; // Power to lower the glyph mechanism gently
    double pl2 = -0.5; // Power to raise the glyph mechanism after it is past the highest
    float currPos; // Current position of the glyph mechanism
    boolean disableArmControl; // Disable software assisted arm control
    boolean enableArmControl; // Enable software assisted arm control
    boolean armControl; // Boolean storing the state of whether software assisted arm control is activated

    @Override
    public void init(){
        // Initializing to the hardware map
        clampServo = hardwareMap.crservo.get("clampServo");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        armControl = true;
    }

    public void loop() {
        // Setting values to gamepad controls
        rt = gamepad1.right_trigger;
        lt = gamepad1.left_trigger;
        rb = gamepad1.right_bumper;
        lb = gamepad1.left_bumper;
        rj = gamepad1.right_stick_y;
        lj = gamepad1.left_stick_y;
        disableArmControl = gamepad1.x;
        enableArmControl = gamepad1.a;

        // Enabling/disabling arm control
        if(disableArmControl){
            armControl = false;
        }

        if(enableArmControl){
            armControl = true;
        }

        if(armControl) {
        //Starting encoder process to get current position of glyph mechanism
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currPos = armMotor.getCurrentPosition();

        // If statement that checks the position of the glyph mechanism and changes the values
        // assignes as powers to the armMotor based off of it. Used to enable smoother control of
        // the glyph mechanism.

            if (currPos <= 2240 / 4 + 5) {
                if (rt > 0.5) {
                    armMotor.setPower(pr1);
                } else if (lt > 0.5) {
                    armMotor.setPower(pl1);
                } else {
                    armMotor.setPower(0);
                }
            } else {
                if (rt > 0.5) {
                    armMotor.setPower(pr2);
                } else if (lt > 0.5) {
                    armMotor.setPower(pl2);
                } else {
                    armMotor.setPower(0);
                }
            }
        } else {
            if (rt > 0.5) {
                armMotor.setPower(pr1);
            } else if (lt > 0.5) {
                armMotor.setPower(pl1);
            } else {
                armMotor.setPower(0);
            }
        }

        // Control of clampServo to enable grabbing blocks
        if(rb){
            clampServo.setPower(1);
        } else if(lb){
            clampServo.setPower(-1);
        } else {
            clampServo.setPower(0);
        }

        // Powers assigned based on joystick values to the drive motors to enable tank drive
        motorRight.setPower(rj);
        motorLeft.setPower(lj);

    }
}
