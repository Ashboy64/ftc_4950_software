package org.firstinspires.ftc.teamcode;

/**
 * Created by rao_a on 11/30/2017.
 */

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class RobotClassFinalUse {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor armMotor;
    ColorSensor colorSensor;
    GyroSensor gyro;
    VuforiaLocalizer vuforia;
    CRServo clampServo;

    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    public RobotClassFinalUse() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        armMotor = ahwMap.dcMotor.get("armMotor");
        gyro = ahwMap.gyroSensor.get("gyro");
        clampServo = ahwMap.crservo.get("clampServo");

        gyro.calibrate();

        while(gyro.isCalibrating()){

        }

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}