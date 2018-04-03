package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by mnawani on 10/31/2017.
 */

public class HelperMethod2 extends LinearOpMode {
    GyroSensor gyro;
    DcMotor leftMotor;
    DcMotor rightMotor;

    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gyro.calibrate();
        while (gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro.");
            telemetry.update();
        }
        waitForStart();
        gyroTurning(90.00);
    }

    public void gyroTurning (double degrees) {
        if (degrees-gyro.getHeading() > 180) {
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
        } else {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }

        while(gyro.getHeading() != degrees) {

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
