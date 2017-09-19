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

public class Carter_TelyOp extends OpMode{
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftJoyStick;
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftJoyStick = hardwareMap.dcMotor.get("leftJoyStick");
    }

    public void loop(){
        if(gamepad1.left_stick_x != 0){
            leftMotor.setPower(gamepad1.left_stick_x * 1);
            rightMotor.setPower(gamepad1.left_stick_x * 1);
        }else{
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    public void stop(){

    }
}
