package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by rao_a on 11/16/2017.
 */

public class ArmTests extends LinearOpMode {
    DigitalChannel tsClosed;
    DigitalChannel tsOpen;

    public void runOpMode(){
        tsClosed = hardwareMap.get(DigitalChannel.class, "tsClosed");
        tsOpen = hardwareMap.get(DigitalChannel.class, "tsOpen");

    }
}
