package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Aayushiron on 9/14/17.
 */

@Autonomous(name="MiniChallengeAayush", group ="Concept")
//@Disabled
public class MiniChallenge_Aayush extends LinearOpMode{

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    DcMotor LeftMotor;
    DcMotor RightMotor;
    ColorSensor colorSensor;
    GyroSensor gyro;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        ElapsedTime opmodeRunTime = new ElapsedTime();


        while (!isStarted()) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds());
            telemetry.update();
            idle();
        }

        LeftMotor = hardwareMap.dcMotor.get("motor1");
        RightMotor = hardwareMap.dcMotor.get("motor2");
        colorSensor = hardwareMap.colorSensor.get("colorSensor1");
        gyro = hardwareMap.gyroSensor.get("gyro1");

        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.calibrate();

        while (gyro.isCalibrating()) {
            telemetry.addData(">", "Calibrating Gyro. Do not touch or the grim Ashish Rao will find you");
            telemetry.update();
        }

        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftMotor.setTargetPosition(3588);
        RightMotor.setTargetPosition(3588);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);

        while (LeftMotor.getCurrentPosition() < LeftMotor.getTargetPosition() && RightMotor.getCurrentPosition() < RightMotor.getTargetPosition()) {

        }

        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colorSensor.enableLed(true);

        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
            telemetry.addData(">", "red");
            telemetry.update();
        } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            telemetry.addData(">", "blue");
            telemetry.update();
        }

        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RightMotor.setPower(-0.5);
        LeftMotor.setPower(0.5);

        while (gyro.getHeading() != 0) {

        }

        while (gyro.getHeading() < 90) {

        }

        RightMotor.setPower(0);
        LeftMotor.setPower(0);

        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftMotor.setTargetPosition(3588);
        RightMotor.setTargetPosition(3588);

        LeftMotor.setPower(1.0);
        RightMotor.setPower(1.0);

        while (LeftMotor.getCurrentPosition() < LeftMotor.getTargetPosition() && RightMotor.getCurrentPosition() < RightMotor.getTargetPosition()) {

        }

        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            for (int i = 0; i<3; i++) {

                relicTemplate = relicTrackables.get(i);

                /**
                 * See if any of the instances of {@link relicTemplate} are currently visible.
                 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
                 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
                 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
                 */
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                    if (pose != null) {

                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        double distance = Math.sqrt(tX*tX + tY*tY + tZ*tZ);

                        telemetry.addData("Distance: ", distance);
                        telemetry.update();

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;

                        telemetry.addData(">", "Picture" + i);
                        telemetry.update();
                    }
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
