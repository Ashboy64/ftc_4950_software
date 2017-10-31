package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Aayushiron on 10/19/17.
 */

@Autonomous(name = "MiniChallenge2Aayush", group = "Concept")
public class MiniChallenge2_Aayush extends LinearOpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    GyroSensor gyro;
    ColorSensor color_sensor;
    VuforiaLocalizer vuforia;
    public int done;
    @Override
    public void runOpMode() throws InterruptedException {


        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        gyro = hardwareMap.gyroSensor.get("gyro");
        color_sensor = hardwareMap.colorSensor.get("colorSensor");

        gyro.calibrate();

        color_sensor.enableLed(true);

        while (gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData(">", "Calibrating Gyro. Do not touch or the grim Ashish Rao will find you");
            telemetry.update();
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWnZ5xz/////AAAAGYmbM16TXEdKscTtfaECY6FzIRnxfc6SV0uwUV+dwPVIWbGyu9567BTp2qzh6ohnawdFrbL290ECRr04ew/QX0Q90SUrGh52+s55yVFPN429A93YJm6AlnV/TEJKb8omxdlqC+Hfy0SLPZSu+UEq9xQMOIfeW+OiRNQyFlUTZNCtQDNuK5jwObgulF83zrexs+c95Cd1jU7PnoX+NgHPjmUWS5H+WVr4yZsewES+oa0jRjGrcGU0/P5USRnqVbKh4976SNjPBGy6fanxJZmQb2Pam56UROtERcdaPDSWg4Nrr0MFlHCvi3PcfyLfdPtBW06JZGWBXu23VJCBQFw3SxGm/IO057P4kbTFti3W5xkU";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor.setPower(-0.5);
        leftMotor.setPower(-0.5);

        while (gyro.getHeading() < 90 && opModeIsActive()) {

        }

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {

            for (int i = 0; i < relicTrackables.size(); i++) {

                relicTemplate = relicTrackables.get(i);

                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
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
                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            rightMotor.setPower(0.5);
                            leftMotor.setPower(0.5);

                            while (gyro.getHeading() > 300 && opModeIsActive()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            leftMotor.setTargetPosition(23091);
                            rightMotor.setTargetPosition(23091);

                            leftMotor.setPower(-1);
                            rightMotor.setPower(1);

                            while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            rightMotor.setPower(0.5);
                            leftMotor.setPower(0.5);

                            while (gyro.getHeading() < 272 && opModeIsActive()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            leftMotor.setPower(-1);
                            rightMotor.setPower(1);

                            while (!(color_sensor.red() > color_sensor.blue() && color_sensor.red() > color_sensor.green()) && opModeIsActive()) {

                            }

                            done = 1;
                            break;

                        } else if (i==1) {
                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            rightMotor.setPower(-0.5);
                            leftMotor.setPower(-0.5);

                            while (gyro.getHeading() > 269 && opModeIsActive()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            leftMotor.setTargetPosition(23091);
                            rightMotor.setTargetPosition(23091);

                            leftMotor.setPower(-1);
                            rightMotor.setPower(1);

                            while (leftMotor.getCurrentPosition() < leftMotor.getTargetPosition() && rightMotor.getCurrentPosition() < rightMotor.getTargetPosition()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            rightMotor.setPower(-0.5);
                            leftMotor.setPower(-0.5);

                            while (gyro.getHeading() < 10 && opModeIsActive()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            leftMotor.setPower(-1);
                            rightMotor.setPower(1);

                            while (!(color_sensor.red() > color_sensor.blue() && color_sensor.red() > color_sensor.green()) && opModeIsActive()) {

                            }

                            done = 1;
                            break;
                        } else if (i==2){
                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            rightMotor.setPower(0.5);
                            leftMotor.setPower(0.5);

                            while (gyro.getHeading() < 315 && opModeIsActive()) {

                            }

                            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            leftMotor.setPower(-1);
                            rightMotor.setPower(1);

                            while (!(color_sensor.red() > color_sensor.blue() && color_sensor.red() > color_sensor.green()) && opModeIsActive()) {

                            }

                            done = 1;
                            break;
                        }
                    }


                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }
            }
            if (done == 1) {
                break;
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
