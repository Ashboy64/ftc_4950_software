package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Aayushiron on 12/8/17.
 */

@TeleOp(name="gyro turning tests", group ="Concept")

public class GyroTurningTests extends OpMode {
    RobotClassFinalUse robot = new RobotClassFinalUse();
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData(">", "Press Play");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;

        if(x){
            gyroTurningx(90);
        } else if(y){
            gyroTurningy(90);
        } if(a){
            gyroTurninga(90);
        }
    }

    public void gyroTurningx (double degrees) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (degrees-robot.gyro.getHeading() > 180) {
            robot.leftMotor.setPower(-0.5);
            robot.rightMotor.setPower(0.5);
        } else {
            robot.leftMotor.setPower(0.5);
            robot.rightMotor.setPower(-0.5);
        }

        while(((robot.gyro.getHeading() < degrees - 5) || (robot.gyro.getHeading() > degrees + 5))) {
            if(gamepad1.b){
                break;
            }
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroTurningy (double degrees) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double target = (robot.gyro.getHeading() + degrees)%360;
        double range = 10;
        while(robot.gyro.getHeading() > target|| robot.gyro.getHeading() < target){
            if (robot.gyro.getHeading() > target) {
                robot.leftMotor.setPower(-0.5);
                robot.rightMotor.setPower(0.5);
            }else if (robot.gyro.getHeading() < target) {
                robot.leftMotor.setPower(0.5);
                robot.rightMotor.setPower(-0.5);
            }

            if(gamepad1.b){
                break;
            }

        }



        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void gyroTurninga (double degrees) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double target = (robot.gyro.getHeading() + degrees)%360;
        double range = 10;
        while(robot.gyro.getHeading() > target|| robot.gyro.getHeading() < target){
            if (robot.gyro.getHeading() > target) {
                robot.leftMotor.setPower(-0.5 * Math.sin(Math.abs(target - robot.gyro.getHeading())) - 0.1);
                robot.rightMotor.setPower(0.5 * (Math.sin(Math.abs(target - robot.gyro.getHeading()))) + 0.1);
            }else if (robot.gyro.getHeading() < target) {
                robot.leftMotor.setPower(0.5 * Math.sin(Math.abs(degrees - target)) + 0.1);
                robot.rightMotor.setPower(-0.5 * Math.sin(Math.abs(degrees - target)) - 0.1);
            }

            if(gamepad1.b){
                break;
            }
        }


        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
