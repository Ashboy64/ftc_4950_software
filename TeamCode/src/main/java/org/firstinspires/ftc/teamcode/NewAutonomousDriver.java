package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by justi on 2018-01-03.
 * this class is used by NewAutonomous for simpler movement and hardware control
 * implement the following methods as described in the comments
 */

public class NewAutonomousDriver {
    private LinearOpMode opMode;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    VuforiaLocalizer vuforia;

    //TODO declaration of motors, servos, sensors, etc.

    public NewAutonomousDriver(HardwareMap hardwareMap, LinearOpMode opMode) {
        //TODO hardware initialization
    }

    /**
     * turns the robot with the gyro
     *
     * @param degrees turns the robot by this angle; positive is clockwise, negative is counterclockwise
     */
    public void turn(int degrees) {
        while (opMode.opModeIsActive()) {
            //TODO
        }
    }

    /**
     * drives the robot using encoders
     *
     * @param inches drives this many inches; positive is forwards, negative is backwards
     */
    public void drive(double inches) {
        while (opMode.opModeIsActive()) {
            //TODO
        }
    }

    /**
     * opens the clamp by running the clamp servo until the open touch sensor is pressed
     */
    public void openClamp() {
        while (opMode.opModeIsActive()) {
            //TODO
        }
    }

    /**
     * sets the jewel arm position
     *
     * @param position the position to set the jewel arm to; 0 is retracted, 1 is lowered
     */
    public void setJewelArmPosition(double position) {
        //TODO
    }

    /**
     * reads the colour of the jewel from the colour sensor
     *
     * @return -1 for a blue jewel, 1 for a red jewel
     */
    public int getJewelColour() {
        //TODO
        return 0;
    }

    /**
     * uses vuforia to read the vision target to determine which column to place the glyph in
     *
     * @return 0 for the left column, 1 for the centre column, 2 for the right column, 3 for unknown
     */
    public int getTargetColumn() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        opMode.telemetry.addData(">", "Press Play to start");
        opMode.telemetry.update();
        opMode.waitForStart();

        relicTrackables.activate();

        while (opMode.opModeIsActive()) {
            relicTemplate = relicTrackables.get(0);

            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                opMode.telemetry.addData("VuMark", vuMark);

                break;

            } else {
                opMode.telemetry.addData("VuMark", "not visible");
            }
        }

        if (vuMark == RelicRecoveryVuMark.CENTER) {
            return 1;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            return 2;
        } else if (vuMark == RelicRecoveryVuMark.LEFT) {
            return 0;
        } else {
            return 3;
        }
    }
}