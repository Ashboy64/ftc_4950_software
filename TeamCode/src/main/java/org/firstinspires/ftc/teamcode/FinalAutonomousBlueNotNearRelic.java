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
 * Created by mnawani on 11/16/2017.
 */

public class FinalAutonomousBlueNotNearRelic extends LinearOpMode {
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
    CRServo jewelServo;
    ColorSensor colorSensor;
    public int trackableViewed;
    double cryptoBoxWidth = 7.63;
    int matLength = 24;
    public boolean seenPicture= false;
    RobotClassFinalUse robot = new RobotClassFinalUse();
    /*
       *******HELPER METHOD DESCRIPTIONS*********
       *armMoving - moves the arm from the resting position to the dropping position and then back again
       *movingForward - takes in a parameter for distance (inches) and then moves the robot forward by the specified distance
       *armGrabbing - allows the robot to move its clamp to a specified position by offsetting from the center
       *gyroTurning - takes in a degree parameter and then turns the robot to the specified degree
     */
    @Override

    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.dcMotor.get("armMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        clampServo = hardwareMap.crservo.get("clampServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        jewelServo = hardwareMap.crservo.get("jewelServo");

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

        gyroTurning(135);

        while (opModeIsActive()) {
//            movingForward(getToJewel);
//            gyroTurning(180.00);
//            if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
//                ElapsedTime opmodeRunTime1 = new ElapsedTime();
//                jewelServo.setPower(1);
//                while (opmodeRunTime1.seconds() < armWaiting) {
//                    telemetry.addData("waiting for arm to get to position", "");
//                    telemetry.update();
//                    idle();
//                }
//                jewelServo.setPower(0);
//                ElapsedTime opModeRunTime2 = new ElapsedTime();
//                jewelServo.setDirection(DcMotorSimple.Direction.REVERSE);
//                jewelServo.setPower(1);
//                while (opModeRunTime2.seconds() < armWaiting) {
//                    telemetry.addData("waiting for arm to get to position", "");
//                    telemetry.update();
//                    idle();
//                }
//                jewelServo.setPower(0);
//                clampServo.setPower(0);
//                clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
//            }

            for (int i = 0; i < relicTrackables.size(); i++) {
                relicTemplate = relicTrackables.get(i);

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                    if (pose != null) {

                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;

                        if (i == 0) {
                            trackableViewed = 0;
                        } else if (i == 1) {
                            trackableViewed = 1;
                        } else {
                            trackableViewed = 2;
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
        gyroTurning(-135);
        movingForward((cryptoBoxWidth/2) + (trackableViewed * cryptoBoxWidth));
        gyroTurning(-107.6100888);
        movingForward(Math.sqrt(
                (cryptoBoxWidth * cryptoBoxWidth) + (matLength * matLength)
        ));
        armRelease();
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void movingForward(double distance) {
        int encoderTicks = (int) Math.ceil((distance/wheel_circumference) * ticksPerRevolution);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(encoderTicks);
        rightMotor.setTargetPosition(encoderTicks);

        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    public void armMoving() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
        while (armMotor.getCurrentPosition() < armMotor.getTargetPosition() && opModeIsActive()) {

        }
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armGrabbing();
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(1120);
        armMotor.setPower(1);
    }

    public void armGrabbing() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting) {
            telemetry.addData("waiting for arm to get to position", "");
            telemetry.update();
            idle();
        }
        clampServo.setPower(0);
    }

    public void armRelease() {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        clampServo.setDirection(DcMotorSimple.Direction.REVERSE);
        clampServo.setPower(1);
        while (opmodeRunTime.seconds() < armWaiting) {
            telemetry.addData("waiting for arm to get to position", "");
            telemetry.update();
            idle();
        }
        clampServo.setPower(0);
        clampServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void gyroTurning (double degrees) {
        if (degrees-gyro.getHeading() > 180) {
            leftMotor.setPower(-0.5);
            rightMotor.setPower(0.5);
        } else {
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }

        while(gyro.getHeading() != degrees) {

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
