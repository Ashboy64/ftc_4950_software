package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Aayushiron on 10/24/17.
 */

@Autonomous(name = "CRServo", group = "Concept")
public class CRServo_Aayush extends LinearOpMode{
    CRServo cr_Servo;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        cr_Servo = hardwareMap.crservo.get("crservo1");
        cr_Servo.setDirection(DcMotorSimple.Direction.REVERSE);
        cr_Servo.setPower(1.0);
        double counterStart = opmodeRunTime.seconds();
        while(opmodeRunTime.seconds() < counterStart + 5){
            telemetry.addData("timings", "In first loop");
            telemetry.update();
        }
        cr_Servo.setPower(0.0);
        cr_Servo.setDirection(DcMotorSimple.Direction.FORWARD);
        cr_Servo.setPower(1.0);
        counterStart = opmodeRunTime.seconds();
        while(opmodeRunTime.seconds() < counterStart + 5){
            telemetry.addData("timings", "daasasdas");
            telemetry.update();
        }
    }
}
