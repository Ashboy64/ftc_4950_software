package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Aayushiron on 9/19/17.
 */

@TeleOp(name="TeleOp", group ="Concept")
//@Disabled
// allows driver to choose between three driver modes explained in further detail below
public class Aayush_TeleOpTest extends OpMode {
    /*
    Hardware map maps the variables in the code to the parts on the physical robot
    motorRight allows for control of the right motor’s actions
    motorLeft allows for control of the left motor’s actions
    state allows for distinction between the three aforementioned driving modes
    State 0 maps to arcade mode
    State 1 maps to tank mode
    State 2 maps to slow mode*/

    DcMotor motorRight;
    DcMotor motorLeft;
    public int state;
    CRServo armClamp;
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("rightMotor");
        motorLeft = hardwareMap.dcMotor.get("leftMotor");
        armClamp = hardwareMap.crservo.get("crServo");
        state = 1;
    }

    @Override
    public void loop() {
        float y = gamepad1.left_stick_y;
        float x = gamepad1.left_stick_x;
        float y2 = gamepad1.right_stick_y;

        modeSwitch();

        if (gamepad1.left_trigger > 0.5) {
            armClamp.setPower(-1.0);
        } else if (gamepad1.right_trigger > 0.5) {
            armClamp.setPower(1.0);
        } else if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0) {
            armClamp.setPower(0.0);
        }

        if (state == 0) {
            arcade(x, y);
        } else if (state == 1) {
            tank(y, y2);
        } else if (state == 2) {
            slow(x, y);
        }

        telemetry.addData("How to change modes: ", "Press ");
    }
    // allows for control of the robot with a single joystick
    // is activated when the X button is pressed on the gamepad
    public void arcade(float x, float y) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setPower(scaleInput((-y-x)));
        motorRight.setPower(scaleInput(-(-y+x)));
    }

    // allows for control of the robot using both joysticks
    // is activated when the Y button is pressed on the gamepad

    public void tank(float y, float y2) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setPower(scaleInput((y*-1)));
        motorRight.setPower(scaleInput(y2));
    }

    // allows for a slower control (half that of arcade mode) of the robot with a single joystick
    // is activated when the B button is pressed on the gamepad
    public void slow(float x, float y) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setPower(scaleInput((-y-x)*0.5));
        motorRight.setPower(scaleInput((-y+x)*-0.5));
    }

    // allows the driver to switch between the three aforementioned driving modes
    // (by pressing X for arcade, Y for tank, and B for slow)
    public void modeSwitch () {
        if (gamepad1.x) {
            state = 0;
        } else if (gamepad1.y) {
            state = 1;
        } else if (gamepad1.b) {
            state = 2;
        }
    }

    // takes an input (for motor power) that is not scaled in a way the computer can interpret correctly
    // then changes it to instructions– from a scale of 0 to 1–  for setting motor power the computer can understand
    public double scaleInput(double dVal)  {

        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        modeSwitch();

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;

        if (dVal < 0) {
            dScale = scaleArray[index];
        } else {
            dScale = -scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
