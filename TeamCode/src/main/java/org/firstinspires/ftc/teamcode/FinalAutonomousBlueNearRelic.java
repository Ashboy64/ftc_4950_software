package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by mnawani on 11/16/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueNearRelic")
public class FinalAutonomousBlueNearRelic extends LinearOpMode{
    VuforiaLocalizer vuforia; //an image-processing library that allows us to analyze pictures
    float getToJewel = 0;
    public int trackableViewed = 1;
    double cryptoBoxWidth = 7.63;
    int matLength = 24;
    public boolean seenPicture= false;
    RobotClassFinalUse robot = new RobotClassFinalUse();
    VuforiaTrackable relicTemplate;

    /*
       *******HELPER METHOD DESCRIPTIONS*********
       *armMoving - moves the arm from the resting position to the dropping position and then back again
       *movingForward - takes in a parameter for distance (inches) and then moves the robot forward by the specified distance
       *armGrabbing - allows the robot to move its clamp to a specified position by offsetting from the center
       *gyroTurning - takes in a degree parameter and then turns the robot to the specified degree
     */
    @Override

    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        robot.init(hardwareMap);

        telemetry.addData(">", "Press Play to start");

        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        robot.gyroTurning(45, this);

        while (opModeIsActive()) {

            for (int i = 0; i < relicTrackables.size(); i++) {

                relicTemplate = relicTrackables.get(i);

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                    if (pose != null) {
                        if (i == 0) {
                            trackableViewed = 1;
                        } else if (i == 1) {
                            trackableViewed = 2;
                        } else {
                            trackableViewed = 3;
                        }
                        seenPicture = true;
                        break;
                    }
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }
            if (seenPicture) {
                break;
            }
        }
        robot.gyroTurning(-45, this);
        robot.movingForward(24, this);
        robot.movingForward((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth), this);
        robot.gyroTurning(-107.6100888, this);
        robot.movingForward(Math.sqrt((cryptoBoxWidth * cryptoBoxWidth) + (matLength * matLength)), this);
        robot.armRelease(this);
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
