package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by rao_a on 11/29/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "#dfdfdf")
public class RobotClassNew extends LinearOpMode{

    RobotClassFinalUse robot = new RobotClassFinalUse();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.gyroTurning(90, this);
    }
}
