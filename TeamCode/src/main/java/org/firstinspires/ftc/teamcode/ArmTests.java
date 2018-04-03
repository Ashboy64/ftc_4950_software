package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by rao_a on 11/16/2017.
 */

@Autonomous(name = "touchSensorTest", group = "Concept")
@Disabled
public class ArmTests extends LinearOpMode {
    DigitalChannel tsClosed;
    DigitalChannel tsOpen;

    public void runOpMode(){
        tsClosed = hardwareMap.get(DigitalChannel.class, "tsClosed");
        tsOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");

        waitForStart();

        tsClosed.setMode(DigitalChannel.Mode.INPUT);
        tsOpen.setMode(DigitalChannel.Mode.INPUT);

        while (opModeIsActive()) {
            telemetry.addData("Closed -> " + tsClosed.getState(), "Open -> ", tsOpen.getState());
            telemetry.update();
        }
    }
}
