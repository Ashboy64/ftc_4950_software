package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Aayushiron on 10/31/17.
 */
// 8.29 inches
public class HelperMethods1 extends LinearOpMode{
    double wheel_circumference;
    double wheel_diameter = 3.5;
    DcMotor leftMotor;
    DcMotor rightMotor;
    int ticksPerRevolution = 2240;
    DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        wheel_circumference = 2 * 3.14 * (wheel_diameter/2);

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor = hardwareMap.dcMotor.get("armMotor");
    }

    public void movingForward(double distance) {
        int encoderTicks = (int) Math.ceil((distance/wheel_circumference) * ticksPerRevolution);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(encoderTicks);
        rightMotor.setTargetPosition(encoderTicks);

        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    public void armMoving() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1050);
        armMotor.setPower(1.0);
        while (armMotor.getCurrentPosition() < armMotor.getTargetPosition() && opModeIsActive()) {

        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armGrabbing();
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1050);
        armMotor.setPower(1.0);
    }

    public void armGrabbing() {

    }
}
