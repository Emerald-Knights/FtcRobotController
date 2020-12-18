package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class robot {
    DcMotor rightFront, rightBack, leftFront, leftBack, spin, launch;
    BNO055IMU imu;
    Orientation angle;
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    VuforiaLocalizer vuforia = null;
    TFObjectDetector tfod;
    Servo leftLift;
    Servo rightLift;

    final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    final String LABEL_FIRST_ELEMENT = "Quad";
    final String LABEL_SECOND_ELEMENT = "Single";
    public void init (HardwareMap hardwareMap, LinearOpMode linearOpMode){
        launch = hardwareMap.get(DcMotor.class, "launch");
        spin = hardwareMap.get(DcMotor.class,"spin");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        //leftLift=hardwareMap.get(Servo.class, "leftLift");
        rightLift=hardwareMap.get(Servo.class, "rightLift");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        angle = imu.getAngularOrientation();
    }
    public void initTF(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AZrZbhj/////AAAAmQF53NIZpEu8twb/VTB2FVWHbBhbMfb3tV+BUuyjHQKpKVPMUg/QLWJr3ZruBFCpSZOIe4Ss5JD6EuVJdnIksAIDC0sxSf9s3JX/QadoDOgzRLZgWF8E19KI6tUD5Anf1QBJNfnzIrrySGgjGyW7GgEvdCzfElATS8DnKN2LY3Fb0+kVp8mrKcEy7SpxUzmllclnYwDXBtkr2e5ad0sbG0JFZkH5A7OMzBZnJFqDAGCqLJLecP5251x+YhJnAK44NKMI3iGHZOZyk318nV6OjEsnPzVf3lSoQ7Ly34cEObjyLbXt9bVsR0fV1ahaW2MrVOeTNF2WniBnHus6IIq6cpL677z7ZmR6NuXtZs8YJQyw";
        ;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    // TODO add in direction later
    public void parking(int ticks, int direction) {
        while (leftFront.getCurrentPosition() < ticks && linearOpMode.opModeIsActive()) {
            if (leftFront.getCurrentPosition() <= ticks/3) {
                leftFront.setPower((leftFront.getCurrentPosition()/ (ticks/3)) * 0.8 + 0.1);
                leftBack.setPower((leftFront.getCurrentPosition()/ (ticks/3)) * 0.8 + 0.1);
                rightFront.setPower((leftFront.getCurrentPosition()/ (ticks/3)) * 0.8 + 0.1);
                rightBack.setPower((leftFront.getCurrentPosition()/ (ticks/3)) * 0.8 + 0.1);

            } else if (leftFront.getCurrentPosition() < 2.0/3 * ticks && ticks/3 < leftFront.getCurrentPosition()) {
                leftFront.setPower(0.8);
                leftBack.setPower(0.8);
                rightFront.setPower(0.8);
                rightBack.setPower(0.8);
            }
            else if (leftFront.getCurrentPosition() >= 2.0/3.0 * ticks){
                leftFront.setPower(((ticks - (leftFront.getCurrentPosition()))/ (ticks/3)) * 0.8 + 0.1);
                leftBack.setPower(((ticks - (leftFront.getCurrentPosition()))/ (ticks/3)) * 0.8 + 0.1);
                rightFront.setPower(((ticks - (leftFront.getCurrentPosition()))/ (ticks/3)) * 0.8 + 0.1);
                rightBack.setPower(((ticks - (leftFront.getCurrentPosition()))/ (ticks/3)) * 0.8 + 0.1);
            }

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double checkAngle(double angle){
        return angle % Math.PI;
    }

    public void turning(double radians){
        double angle = imu.getAngularOrientation().firstAngle;
        while(angle < radians) {
            angle = imu.getAngularOrientation().firstAngle;
            if (angle <= radians) {
                leftFront.setPower(-((angle / (radians/3)) * 0.8 + 0.1));
                leftBack.setPower(-((angle/ (radians/3)) * 0.8 + 0.1));
                rightFront.setPower((angle/ (radians/3)) * 0.8 + 0.1);
                rightBack.setPower((angle/ (radians/3)) * 0.8 + 0.1);

            } else if (angle < 2.0/3 * radians && radians/3 < leftFront.getCurrentPosition()) {
                leftFront.setPower(-0.8);
                leftBack.setPower(-0.8);
                rightFront.setPower(0.8);
                rightBack.setPower(0.8);

            }
            else if (angle >= 2.0/3.0 * radians){
                leftFront.setPower(-(((radians - (angle))/ (radians/3)) * 0.8 + 0.1));
                leftBack.setPower(-(((radians - (angle))/ (radians/3)) * 0.8 + 0.1));
                rightFront.setPower(((radians - (angle))/ (radians/3)) * 0.8 + 0.1);
                rightBack.setPower(((radians - (angle))/ (radians/3)) * 0.8 + 0.1);
            }
        }
    }


}
