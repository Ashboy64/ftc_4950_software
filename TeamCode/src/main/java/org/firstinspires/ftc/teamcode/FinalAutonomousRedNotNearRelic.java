package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Created by Aayushiron on 11/21/17.
 */

public class FinalAutonomousRedNotNearRelic extends LinearOpMode {
    double wheel_diameter = 3.5; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    DcMotor leftMotor; //allows for control of the robot’s left motor’s actions
    DcMotor rightMotor; //allows for control of the robot’s right motor’s actions
    int ticksPerRevolution = 2240; //the amount of ticks the encoder takes to revolve one wheel
    DcMotor armMotor; //allows for control of our robot’s arm
    GyroSensor gyro; //receives information about the direction of our robot
    VuforiaLocalizer vuforia; //an image-processing library that allows us to analyze pictures
    CRServo clampServo;
    double armWaiting = 2.0;
    float getToJewel = 0;
   // CRServo jewelServo;
    ColorSensor colorSensor;
    int trackableViewed;
    double robotLength = 0;
    double glyphLength = 6;
    double deg = 80.96792587;
    double thirdDistance = 33.13103681;
    double secondDistance =  27.92762253;
    double firstDistance =  24.20052892;
    RobotClassFinalUse robot = new RobotClassFinalUse();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        gyro.calibrate();
        while (gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro.");
            telemetry.update();
        }

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.movingForward(getToJewel, this);
        robot.gyroTurning(180.00, this);
//
//        if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.teamColour()) {
//            ElapsedTime opmodeRunTime = new ElapsedTime();
//            while (opmodeRunTime.seconds() < armWaiting) {
//                telemetry.addData("waiting for arm to get to position", "");
//                telemetry.update();
//                idle();
//            }
//            ElapsedTime opModeRunTime = new ElapsedTime();
//            while (opmodeRunTime.seconds() < armWaiting) {
//                telemetry.addData("waiting for arm to get to position", "");
//                telemetry.update();
//                idle();
//            }
//            clampServo.setPower(0);
//            clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
//        }


        while (opModeIsActive()) {
            robot.gyroTurning(-45, this);

            RelicRecoveryVuMark vuMark;

        }
        robot.gyroTurning(45, this);
        robot.movingForward(24 - (24/3)*trackableViewed, this);
        robot.gyroTurning(deg, this);
        if(trackableViewed == 3) {
            robot.movingForward(thirdDistance, this);
        }else if(trackableViewed == 2){
            robot.movingForward(secondDistance, this);
        }else {
            robot.movingForward(firstDistance, this);
        }
        robot.armRelease(this);
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
