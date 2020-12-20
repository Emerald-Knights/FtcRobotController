package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
    DcMotor upwards;

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
        leftLift=hardwareMap.get(Servo.class, "leftLift");
        rightLift=hardwareMap.get(Servo.class, "rightLift");

        upwards=hardwareMap.get(DcMotor.class, "upwards");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection((DcMotorSimple.Direction.REVERSE));
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(Servo.Direction.REVERSE);
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
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void move(int ticks, int direction) {
        DcMotor m= rightBack;
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(m.getCurrentPosition()) < Math.abs(ticks) && linearOpMode.opModeIsActive()) {
            if (Math.abs(m.getCurrentPosition()) <= Math.abs(ticks/3.0)) {
                leftFront.setPower(direction *((Math.abs(m.getCurrentPosition())/ Math.abs(ticks/3.0)) * 0.5 + 0.3));
                leftBack.setPower(direction * ((Math.abs(m.getCurrentPosition())/ Math.abs(ticks/3.0)) * 0.5 + 0.3));
                rightFront.setPower(direction * ((Math.abs(m.getCurrentPosition())/ Math.abs(ticks/3.0)) * 0.5 + 0.3));
                rightBack.setPower(direction * ((Math.abs(m.getCurrentPosition())/ Math.abs(ticks/3.0)) * 0.5 + 0.3));
                //telemetry.addData("rightFront:" + rightFront.getCurrentPosition());
                //telemetry.addData("rightFront pow: "+ rightFront.getPower());

            } else if (Math.abs(m.getCurrentPosition() )< 2.0/3 * Math.abs(ticks) && Math.abs(ticks/3.0) < Math.abs(m.getCurrentPosition())) {
                leftFront.setPower(direction * (0.8));
                leftBack.setPower(direction * (0.8));
                rightFront.setPower(direction * (0.8));
                rightBack.setPower(direction * (0.8));
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                }
            }
            else if (Math.abs(m.getCurrentPosition()) >= 2.0/3.0 * Math.abs(ticks)){
                leftFront.setPower(direction * (Math.abs(Math.abs(ticks) - Math.abs(m.getCurrentPosition())/ Math.abs(ticks/3.0)) * 0.5 + 0.3));
                leftBack.setPower(direction * (Math.abs( ( Math.abs(ticks) - Math.abs(m.getCurrentPosition() ) ) / Math.abs(ticks/3.0) ) * 0.5 + 0.3));
                rightFront.setPower(direction * (Math.abs((Math.abs(ticks) - Math.abs(m.getCurrentPosition()))/ Math.abs((ticks/3.0))) * 0.5 + 0.3));
                rightBack.setPower(direction * (Math.abs((Math.abs(ticks) - Math.abs(m.getCurrentPosition()))/ Math.abs((ticks/3.0))) * 0.5 + 0.3));
            }
            linearOpMode.telemetry.addData("rightFront:",  rightFront.getPower());
            linearOpMode.telemetry.addData("leftFront:",  leftFront.getPower());
            linearOpMode.telemetry.addData("pos", m.getCurrentPosition());
            linearOpMode.telemetry.addData("ticks",  ticks);
            linearOpMode.telemetry.addData("calculation", direction * (((Math.abs(ticks) - Math.abs(m.getCurrentPosition()))/ Math.abs((ticks/3.0))) * 0.5));
            linearOpMode.telemetry.update();

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafe(int ticks, int direction) {
        DcMotor m=rightBack;
        while (Math.abs(m.getCurrentPosition()) < ticks && linearOpMode.opModeIsActive()) {
            if (Math.abs(m.getCurrentPosition()) <= ticks/3.0) {
                leftFront.setPower(-direction *((Math.abs(m.getCurrentPosition())/ (ticks/3.0)) * 0.8 + 0.1));
                leftBack.setPower(direction * ((Math.abs(m.getCurrentPosition())/ (ticks/3.0)) * 0.8 + 0.1));
                rightFront.setPower(direction * ((Math.abs(m.getCurrentPosition())/ (ticks/3.0)) * 0.8 + 0.1));
                rightBack.setPower(-direction * ((Math.abs(m.getCurrentPosition())/ (ticks/3.0)) * 0.8 + 0.1));
                //linearOpMode.telemetry.addData("m:", m.getCurrentPosition());
                //telemetry.update();
                //try {
                //Thread.sleep(10);
                //} catch (InterruptedException e) {
                //}
            } else if (Math.abs(m.getCurrentPosition() )< 2.0/3 * ticks && ticks/3.0 < Math.abs(m.getCurrentPosition())) {
                leftFront.setPower(-direction * (0.8));
                leftBack.setPower(direction * (0.8));
                rightFront.setPower(direction * (0.8));
                rightBack.setPower(-direction * (0.8));
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                }
            }
            else if (Math.abs(m.getCurrentPosition()) >= 2.0/3.0 * ticks){
                leftFront.setPower(-direction * (((ticks - Math.abs(m.getCurrentPosition()))/ (ticks/3.0)) * 0.8 + 0.1));
                leftBack.setPower(direction * (((ticks - Math.abs(m.getCurrentPosition()))/ (ticks/3.0)) * 0.8 + 0.1));
                rightFront.setPower(direction * (((ticks - Math.abs(m.getCurrentPosition()))/ (ticks/3.0)) * 0.8 + 0.1));
                rightBack.setPower(-direction * (((ticks - Math.abs(m.getCurrentPosition()))/ (ticks/3.0)) * 0.8 + 0.1));
            }
            linearOpMode.telemetry.addData("leftFront:", Math.abs(m.getCurrentPosition()));
            linearOpMode.telemetry.update();

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double checkAngle(double angle){
        return angle % Math.PI;
    }

    public void turning(double radians){
        double angle = imu.getAngularOrientation().firstAngle;
        int direction = 1;
        while(Math.abs(angle) < Math.abs(radians)- 0.1) {
            angle = imu.getAngularOrientation().firstAngle;
            if (checkAngle(radians - angle) < 0){
                direction = -1;
            }
            else if (checkAngle(radians - angle) > 0){
                direction = 1;
            }
            if (angle <= radians/3) {
                leftFront.setPower(direction * (-( Math.abs(angle / (radians/3.0)) * 0.8 + 0.1)));
                leftBack.setPower (direction * (-( Math.abs(angle/ (radians/3.0)) * 0.8 + 0.1)));
                rightFront.setPower(direction * ( Math.abs(angle/ (radians/3.0)) * 0.8 + 0.1));
                rightBack.setPower(direction * ( Math.abs(angle/ (radians/3.0)) * 0.8 + 0.1));
                //telemetry.addData("Angle:" + (angle / (radians/3)));

                //try {
                //Thread.sleep(10);
                //} catch (InterruptedException e) {
                //}
            }
            else if (angle < 2.0/3 *radians && radians/3.0 < angle) {
                leftFront.setPower(direction * (-0.8));
                leftBack.setPower(direction * (-0.8));
                rightFront.setPower(direction * (0.8));
                rightBack.setPower(direction * (0.8));
//                try {
//                    Thread.sleep(10);
//                } catch (InterruptedException e) {
//                }
            }
            else if (angle >= 2.0/3.0 * radians){
                leftFront.setPower(direction * (-((Math.abs( checkAngle(radians - (angle) / (radians/3.0)))) * 0.8 + 0.1)));
                leftBack.setPower(direction * (-((Math.abs (checkAngle(radians - (angle)/ (radians/3.0)))) * 0.8 + 0.1)));
                rightFront.setPower(direction * ((Math.abs(checkAngle(radians - (angle)/ (radians/3.0)))) * 0.8 + 0.1));
                rightBack.setPower(direction * ((Math.abs(checkAngle(radians - (angle)/ (radians/3.0)))) * 0.8 + 0.1));
            }
            //telemetry.addData("Angle:" + angle);
            //telemetry.addData("Left:" + leftFront.getPower());
            //telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



}
