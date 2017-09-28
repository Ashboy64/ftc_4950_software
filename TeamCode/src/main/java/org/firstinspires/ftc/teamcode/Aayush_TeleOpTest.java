package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Aayushiron on 9/19/17.
 */

@TeleOp(name="tele_op_1_arcade", group ="Concept")
//@Disabled
public class Aayush_TeleOpTest extends OpMode {
    DcMotor motorRight;
    DcMotor motorLeft;
    public int state;
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("rightmotor");
        motorLeft = hardwareMap.dcMotor.get("leftmotor");
    }

    @Override
    public void loop() {
        float y = gamepad1.left_stick_y;
        float x = gamepad1.left_stick_x;
        float y2 = gamepad1.right_stick_y;
        state = 0;

        modeSwitch();

        if (state == 0) {
            arcade(x, y);
        } else if (state == 1) {
            tank(y, y2);
        } else if (state == 2) {
            slow(x, y);
        }
    }

    public void arcade(float x, float y) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setPower(scaleInput(-y-x));
        motorRight.setPower(scaleInput(-y+x));

        modeSwitch();

        /*if (x > 0.0) {
            motorLeft.setPower(scaleInput((y * -1) + (x*-1)));
            motorRight.setPower(scaleInput(y * -1));
        } else if (x < 0.0) {
            motorRight.setPower(scaleInput((y * -1) + (x*-1)));
            motorLeft.setPower(scaleInput(y * -1));
        } else {
            motorRight.setPower(scaleInput(y * -1));
            motorLeft.setPower(scaleInput(y * -1));
        } */
    }

    public void tank(float y, float y2) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setPower(scaleInput(y*-1));
        motorRight.setPower(scaleInput(y2*-1));

        modeSwitch();
    }

    public void slow(float x, float y) {
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (x > 0.0) {
            motorLeft.setPower(scaleInput(((y * -1) + (x*-1))*0.25));
            motorRight.setPower(scaleInput((y * -1) * 0.25));
        } else if (x < 0.0) {
            motorRight.setPower(scaleInput(((y * -1) + (x*-1))*0.25));
            motorLeft.setPower(scaleInput((y * -1) * 0.25));
        } else {
            motorRight.setPower(scaleInput((y * -1) * 0.25));
            motorLeft.setPower(scaleInput((y * -1) * 0.25));
        }

        modeSwitch();
    }

    public void modeSwitch () {
        if (gamepad1.x) {
            state = 0;
        } else if (gamepad1.y) {
            state = 1;
        } else if (gamepad1.b) {
            state = 2;
        }
    }

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
