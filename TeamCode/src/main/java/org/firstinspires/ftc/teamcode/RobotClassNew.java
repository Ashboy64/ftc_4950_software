package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by rao_a on 11/29/2017.
 */

public class RobotClassNew {
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public GyroSensor gyro = null;
    public DcMotor armMotor = null;
    public CRServo clampServo = null;
    public VuforiaLocalizer vuforiaLocalizer = null;

    public void robot(){
    }

    public void config(HardwareMap hardwareMap){
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clampServo = hardwareMap.crservo.get("clampServo");
    }
}
