package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by yinghuang on 9/19/17.
 */

@TeleOp(name = "swag", group = "Concept")
//@Disabled
public class Carter_TelyOp extends OpMode{
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftmotor = hardwareMap.dcMotor.get("leftmotor");
        rightmotor = hardwareMap.dcMotor.get("rightmotor");
    }
    @Override
    public void loop(){
        if(gamepad1.left_stick_x != 0){
            leftmotor.setPower(scaleInput(gamepad1.left_stick_x * 1));
            rightmotor.setPower(scaleInput(gamepad1.left_stick_x * 1));
        }else{
            leftmotor.setPower(0);
            rightmotor.setPower(0);
        }
    }

    public void stop(){

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

