package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by rao_a on 11/14/2017.
 */

@TeleOp(name="FinalTeleOp", group ="Concept")
public class FinalTele extends OpMode{
    CRServo clampServo; // Servo that controls the clamping motion of the glyph mechanism
    DcMotor armMotor; // Motor used to raise/lower glyph mechanism
    DcMotor motorRight; // Right driving motor
    DcMotor motorLeft; // Left driving motor
    DigitalChannel tsClosed; // Touch Sensor to check when arm is Closed
    DigitalChannel tsOpen; // Touch Sensor to check when arm is Open
    float rt; // Right trigger, raises arm when pressed
    float lt; // Left trigger, lowers arm when pressed
    boolean rb; // Right bumper, opens clamp when pressed
    boolean lb; // Left bumper, closes clamp when pressed
    float rj; // Right joystick, controls right motor
    float lj; // Left joystick, controls left motor
    float rt2; // Right trigger of gamepad 2, raises arm when pressed
    float lt2; // Left trigger of gamepad 2, lowers arm when pressed
    boolean rb2; // Right bumper of gamepad 2, opens clamp when pressed
    boolean lb2; // Left bumper of gamepad 2, closes clamp when pressed
    float rj2; // Right joystick of gamepad 2, controls right motor
    float lj2; // Left joystick of gamepad 2, controls left motor
    double pr1 = 0.5; // Power to raise the glyph mechanism while it's position is before the highest
    double pr2 = -0.05; // Power to raise the glyph mechanism while it's position is past the highest
    double pl1 = 0.05; // Power to lower the glyph mechanism gently
    double pl2 = -0.75; // Power to raise the glyph mechanism after it is past the highest
    double ticksPerRev = 2240; // Encoder ticks in one revolution
    float currPos; // Current position of the glyph mechanism
    boolean armControlButton; // Enable/Disable software assisted arm control
    boolean armControlButton2; // Enable software assisted arm control
    boolean bpressed1; // B button enabling backwards drive pressed on gamepad 1
    boolean bpressed2; // B button enabling backwards drive pressed on gamepad 2

    @Override
    public void init(){
        // Initializing to the hardware map
        clampServo = hardwareMap.crservo.get("clampServo");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        tsClosed = hardwareMap.get(DigitalChannel.class, "tsClosed");
        tsOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop() {
        // Setting values to gamepad controls
        rt = gamepad1.right_trigger;
        lt = gamepad1.left_trigger;
        rb = gamepad1.right_bumper;
        lb = gamepad1.left_bumper;
        rj = gamepad1.right_stick_y;
        lj = gamepad1.left_stick_y;
        rt2 = gamepad2.right_trigger;
        lt2 = gamepad2.left_trigger;
        rb2 = gamepad2.right_bumper;
        lb2 = gamepad2.left_bumper;
        rj2 = gamepad2.right_stick_y;
        lj2 = gamepad2.left_stick_y;
        armControlButton = gamepad1.x;
        armControlButton2 = gamepad2.x;
        bpressed1 = gamepad1.b;
        bpressed2 = gamepad2.b;

        // Touch sensor test
        tsClosed.setMode(DigitalChannel.Mode.INPUT);
        tsOpen.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Closed -> " + tsClosed.getState(), "Open -> ", tsOpen.getState());
        telemetry.update();

        if(!bpressed1 || !bpressed2){  // If the robot is not being driven in reverse
            motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

            if(rj!=0 || lj!=0){
                motorLeft.setPower(lj);
                motorRight.setPower(rj);
            } else if(rj2!=0 || lj2!=0){
                motorLeft.setPower(lj2);
                motorRight.setPower(rj2);
            } else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }

        } else { // Robot driven in reverse, so motor directions reversed and joysticks assigned to motors switch
            motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

            if(rj!=0 || lj!=0){
                motorLeft.setPower(rj);
                motorRight.setPower(lj);
            } else if(rj2!=0 || lj2!=0){
                motorLeft.setPower(rj2);
                motorRight.setPower(lj2);
            } else {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
        }

        // Control of glyph mechanism to move the arm up and down
        if (rt > 0.5 || rt2 > 0.5) {
            armMotor.setPower(pr1); // Set arm to a large positive power
        } else if (lt > 0.5 || lt2 > 0.5) {
            armMotor.setPower(-0.5); // Set arm to a small positive value
        } else {
            armMotor.setPower(0); // Set arm power to 0
        }

        // Control of clampServo to enable grabbing blocks
        if((rb || rb2)){
            clampServo.setPower(1); // Open clamp
        } else if((lb || lb2)){
            clampServo.setPower(-1); // Close clamp
        } else {
            clampServo.setPower(0); // Set clamp power to 0
        }

    }
}
