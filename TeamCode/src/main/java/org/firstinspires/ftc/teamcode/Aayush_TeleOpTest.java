package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by Aayushiron on 9/19/17.
 */

@TeleOp(name="Concept: VuMark Id", group ="Concept")
@Disabled
public class Aayush_TeleOpTest extends OpMode {
    DcMotor motorRight;
    DcMotor motorLeft;
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("right_drive");
        motorLeft = hardwareMap.dcMotor.get("left_drive");
    }

    @Override
    public void loop() {
        float yvert = gamepad1.left_stick_y;

        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorRight.setPower(yvert);
        motorLeft.setPower(yvert);
    }
}
