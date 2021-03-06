package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by mnawani on 9/19/17.
 */

@TeleOp(name="Manvi_TeleOpTest", group ="Concept")
@Disabled
//@Disabled
public class Manvi_TeleOpTest extends OpMode {
    DcMotor motorRight;
    DcMotor motorLeft;
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("rightmotor");
        motorLeft = hardwareMap.dcMotor.get("leftmotor");
    }

    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (x > 0.0) {
            motorLeft.setPower(scaleInput((y * -1) + (x * -1)));
            motorRight.setPower(scaleInput(y * -1));
        } else if (x < 0.0) {
            motorRight.setPower(scaleInput((y * -1) + (x * -1)));
            motorLeft.setPower(scaleInput(y * -1));
        } else {
            motorRight.setPower(scaleInput(y * -1));
            motorLeft.setPower(scaleInput(y * -1));
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