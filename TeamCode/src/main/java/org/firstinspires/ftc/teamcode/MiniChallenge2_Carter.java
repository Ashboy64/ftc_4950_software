package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;


/**
 * Created by yinghuang on 10/12/17.
 */
@Autonomous(name = "MiniChallenge2", group = "concept")
public class MiniChallenge2_Carter extends LinearOpMode {
    GyroSensor gyro;
    ColorSensor color_sensor;
    VuforiaLocalizer vuforia;
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime opmodeRunTime = new ElapsedTime();
        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        color_sensor = hardwareMap.colorSensor.get("colorSensor");

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();

        while (gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro.");
            telemetry.update();
        }

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setPower(-0.5);
        leftMotor.setPower(0.5);

        while (gyro.getHeading() != 90 && opModeIsActive()) {
        }

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            for (int i = 0; i < 3; i++) {
                relicTemplate = relicTrackables.get(i);
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
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

                        double distance = Math.sqrt(tX * tX + tY * tY + tZ * tZ);
                        telemetry.addData("Distance: ", distance);
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;

                        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rightMotor.setPower(-0.5);
                        leftMotor.setPower(0.5);
                        if(i == 0){
                            while (gyro.getHeading() != 0 && opModeIsActive()) {
                            }
                        }else if(i == 1){
                            while(gyro.getHeading() != 270 && opModeIsActive()){
                            }
                        }else{
                            while(gyro.getHeading() != 315 && opModeIsActive()){
                            }
                        }
                        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightMotor.setPower(0);
                        leftMotor.setPower(0);
                        break;
                    }
                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }
            telemetry.update();
            break;
        }
    }
}