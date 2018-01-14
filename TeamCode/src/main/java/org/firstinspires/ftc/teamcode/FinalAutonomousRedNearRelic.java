package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by Aayushiron on 10/31/17.
 */

@Autonomous(name = "FinalAutononousRedNearRelic", group = "Concept")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class FinalAutonomousRedNearRelic extends LinearOpMode{
    double wheel_diameter = 3.5; //the diameter of the wheels of our robot.
    double wheel_circumference = Math.PI * wheel_diameter; //the value of π times the wheel diameter
    int ticksPerRevolution = 2240; //the amount of ticks the encoder takes to revolve one wheel
    DcMotor armMotor; //allows for control of our robot’s arm
    GyroSensor gyro; //receives information about the direction of our robot
    VuforiaLocalizer vuforia; //an image-processing library that allows us to analyze pictures
    double armWaiting = 2.0;
    float getToJewel = 0;
    int trackableViewed;
    double deg = 72.38991115;
    double distanceToMove = 25.18366366;
    RelicRecoveryVuMark vuMark;
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
        robot.init(hardwareMap);

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

        while (opModeIsActive()) {
            robot.movingForward(getToJewel, this);

          //  if (colorSensor.teamColour() > colorSensor.green() && colorSensor.teamColour() > colorSensor.red()) {
                ElapsedTime opmodeRunTime = new ElapsedTime();
  //              jewelServo.setPower(1);
                while (opmodeRunTime.seconds() < armWaiting) {
                    telemetry.addData("waiting for arm to get to position", "");
                    telemetry.update();
                    idle();
                }
    //            jewelServo.setPower(0);
                ElapsedTime opModeRunTime = new ElapsedTime();
      //          jewelServo.setDirection(DcMotorSimple.Direction.REVERSE);
        //        jewelServo.setPower(1);
                while (opmodeRunTime.seconds() < armWaiting) {
                    telemetry.addData("waiting for arm to get to position", "");
                    telemetry.update();
                    idle();
                }
         //   }

            robot.movingForward(8.5, this);
            robot.gyroTurning(90.00, this);

            while (opModeIsActive()) {
                relicTemplate = relicTrackables.get(0);

                vuMark = RelicRecoveryVuMark.from(relicTemplate);

                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    telemetry.addData("VuMark", vuMark);

                    break;

                } else {
                    telemetry.addData("VuMark", "not visible");
                }
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                trackableViewed = 2;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                trackableViewed = 3;
            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
                trackableViewed = 1;
            }
         //   gyroTurning(-90.00);
        //    movingForward(27.50);
         //   gyroTurning(-90.00);
            // add depositing cube code here
         //   movingForward(-2);
            robot.movingForward(24 - 3.815 +  7.63*trackableViewed, this);
            robot.gyroTurning(deg, this);
            robot.movingForward(distanceToMove, this);
            robot.armRelease(this);
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
