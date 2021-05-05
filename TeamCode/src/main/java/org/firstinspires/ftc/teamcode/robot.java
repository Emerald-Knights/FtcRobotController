package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.*;

public class robot {
    public static double endX=0;
    public static double endY=0;
    public static double endAngle;


    public static Point goal =new Point(0,0);
    public static Point redGoal = new Point(72, -36);

    DcMotor rightFront, rightBack, leftFront, leftBack, spin;
    DcMotor leftOdo, rightOdo, horizontalOdo;
    DcMotor grabArm;
    DcMotorEx launch;
    BNO055IMU imu;
    Orientation angle;
    LinearOpMode linearOpMode;
    HardwareMap hardwareMap;
    VuforiaLocalizer vuforia = null;
    TFObjectDetector tfod;
    Servo leftLift;
    Servo rightLift;
    Servo grabber, flippyFlip;
    DcMotor upwards;

    Position location;
    OpenCvWebcam cam;

    ringPipeline pipeline;

    ElapsedTime runtime= new ElapsedTime();

    //List<CurvePoint> path = new ArrayList<>();

    DcMotor[] driveTrain;
    DcMotor[] odo;

    private int currentPath=0;

    final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    final String LABEL_FIRST_ELEMENT = "Quad";
    final String LABEL_SECOND_ELEMENT = "Single";

    public void init (HardwareMap hardwareMap, LinearOpMode linearOpMode){
        launch = (DcMotorEx) hardwareMap.get(DcMotor.class, "launch");
        spin = hardwareMap.get(DcMotor.class,"spin");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        //grabArm = hardwareMap.get(DcMotor.class, "grabArm");
        driveTrain = new DcMotor[]{leftFront, leftBack, rightBack, rightFront};

        //leftLift=hardwareMap.get(Servo.class, "leftLift");
        //rightLift=hardwareMap.get(Servo.class, "rightLift");
        grabber = hardwareMap.get(Servo.class, "grabber");
        flippyFlip = hardwareMap.get(Servo.class, "flippyFlip");

        leftOdo= hardwareMap.get(DcMotor.class, "upwards");
        rightOdo = hardwareMap.get(DcMotor.class, "leftFront");
        horizontalOdo = hardwareMap.get(DcMotor.class, "rightBack");

        odo = new DcMotor[]{leftOdo, rightOdo, horizontalOdo};

        upwards=hardwareMap.get(DcMotor.class, "upwards");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection((DcMotorSimple.Direction.REVERSE));
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightLift.setDirection(Servo.Direction.REVERSE);
        this.linearOpMode = linearOpMode;
        this.hardwareMap = hardwareMap;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.getAngularOrientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        angle = imu.getAngularOrientation();

        PIDFCoefficients launchPID= launch.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        launchPID.p=36;
        launchPID.d=30;

        for(DcMotor pod: odo){
            pod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for(DcMotor pod: odo){
            pod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        location= new Position(0, 0, angle.firstAngle, leftOdo, rightOdo, horizontalOdo, imu, linearOpMode);
        location.start();
    }
    public void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ringPipeline();
        cam.setPipeline(pipeline);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });
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

    public double getY(){
        return location.y;
    }
    public double getX(){
        return location.x;
    }
    public double getHeading(){
        return location.heading;
    }
    public CurvePoint getPosition(){
        return new CurvePoint(getX(), getY(), getHeading());
    }

    public double getRate(){
        //return .0136904762 * (Math.hypot(redGoal.x-getX(), redGoal.y-getY()))+3.542857143;
        double distance = Math.hypot(redGoal.x-getX(), redGoal.y-getY());

        if(Math.abs(distance) < 90){
            return 8.8;
            //return 9.0;
        }

        return .020933333333333 * (distance) + 6.590;
    }

    public double getAngleDiff(){
        //return angleWrap(getHeading() +Math.PI - Math.atan2(redGoal.y-getY(), redGoal.x-getX()));
        return angleWrap(getHeading() + 2.8 - Math.atan2(redGoal.y-getY(), redGoal.x-getX())); //2.85 //95
    }

    public void upward(){
        upwards.setPower(-0.8);
    }
    public void collect(){
        spin.setPower(-0.8);
    }
    public void reverseCollect(){
        spin.setPower(0.8);

    }


/*
    public void move(int ticks, int direction) {
        DcMotor m= rightBack;
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(m.getCurrentPosition()) < Math.abs(ticks) && linearOpMode.opModeIsActive()) {
            leftFront.setPower(direction * (Math.abs(Math.abs(ticks) - Math.abs(m.getCurrentPosition())/ Math.abs(ticks) * 0.8)));
            leftBack.setPower(direction * (Math.abs(Math.abs(ticks) - Math.abs(m.getCurrentPosition())/ Math.abs(ticks) * 0.8)));
            rightFront.setPower(direction * (Math.abs(Math.abs(ticks) - Math.abs(m.getCurrentPosition())/ Math.abs(ticks) * 0.8)));
            rightBack.setPower(direction * (Math.abs(Math.abs(ticks) - Math.abs(m.getCurrentPosition())/ Math.abs(ticks) * 0.8)));

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

 */


    /** Auton
     *

     */
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

            } else if (Math.abs(m.getCurrentPosition() )< 2.0/3 * Math.abs(ticks) && Math.abs(ticks) < Math.abs(m.getCurrentPosition())) {
                leftFront.setPower(direction * (0.8));
                leftBack.setPower(direction * (0.8));
                rightFront.setPower(direction * (0.8));
                rightBack.setPower(direction * (0.8));
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


    public void turning(double radians){
        double angle = imu.getAngularOrientation().firstAngle;
        int direction = 1;
        while(Math.abs(angle) < Math.abs(radians)- 0.1 && linearOpMode.opModeIsActive()) {
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
    public void moveToPosition(double x, double y, double speed, double endAngle){
        double xDis=x-getX(); //x distance to point global
        double yDis=y-getY(); //y distance to point global

        double totalDis=Math.hypot(xDis,yDis); //total distance to point global
        double angleToPos=Math.atan2(yDis, xDis); //angle from robot to point
        //double relativeAngle=angleWrap(getHeading()-angleWrap(-angleToPos+Math.PI/2)); //angle robot is facing from where path ends
        double relativeAngle=angleWrap(angleToPos-getHeading()+Math.PI/2);
        //double relativeAngle=angleWrap(getHeading()- angleToPos);

        double relativeX=Math.cos(relativeAngle)*totalDis; //x velocity robot needs to get to locaction, robot point
        double relativeY=Math.sin(relativeAngle)*totalDis; //y velocity robot needs to get to location, robot point

        double xPower=Math.sqrt(2)*relativeX/(Math.abs(relativeX)+Math.abs(relativeY)); //setting speeds
        double yPower=Math.sqrt(2)*relativeY/(Math.abs(relativeX)+Math.abs(relativeY));

        double turnAngle=angleWrap(endAngle-getHeading()); //amount u want to turn
        //double turnPower=turnAngle*speed;
        double turnPower=0;
        if(Math.abs(turnAngle)>.01){
            turnPower= -turnAngle/Math.abs(turnAngle)*( Math.abs(turnAngle)/Math.PI/2.0+ .3);
        }

        for(int i=0; i<driveTrain.length; i++){
            driveTrain[i].setPower(drive(xPower,yPower,turnPower)[i]*speed);
        }
        //position.xPow=xPower;
        //position.yPow=yPower;
        //position.anglePow=turnPower;

    }

    public double[] drive(double lx, double ly, double rx){
        double lf = ly + rx + lx;
        double lb = ly + rx - lx;
        double rf = ly - rx - lx;
        double rb = ly - rx + lx;


        double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
        double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
        double ratio = magnitude / max;
        if (max == 0) {
            ratio=0;
        }
        double[] powers = {lf*ratio, lb*ratio, rb*ratio, rf*ratio};
        return powers;
    }

    public CurvePoint getFollowPointPath(List<CurvePoint> pathPoints, double xPosRobot, double yPosRobot, double followRadius){

        CurvePoint follow=new CurvePoint(pathPoints.get(currentPath));
        int previousC=currentPath;
        for(int i=currentPath; i<previousC+2 && i<pathPoints.size()-1; i++){
            CurvePoint start=pathPoints.get(i);
            CurvePoint end=pathPoints.get(i+1);

            List<Point> intersections=proximityPathIntersects(new Point(xPosRobot, yPosRobot), followRadius, start.toPoint(), end.toPoint());
            if(intersections.size()>0){
                follow.setPoint(intersections.get(0));
                follow.heading=end.heading;
                currentPath=i;

                //double closestAngle=Integer.MAX_VALUE;
                for(Point intersect: intersections){
                    if(Math.abs(distanceToPoint(intersect.x, intersect.y, end.x, end.y)) < Math.abs(distanceToPoint(follow.x, follow.y, end.x, end.y))){
                        follow.setPoint(intersect);
                    }
                }
            }

        }
        return follow;

    }

    public void followCurve(List<CurvePoint> allPoints, double followDistance, double speed){

        CurvePoint follow=getFollowPointPath(allPoints, getX(), getY(), followDistance);
        moveToPosition(follow.x, follow.y, speed, follow.heading);
    }

    public void followCurveSync(List<CurvePoint> allPoints, double followDistance, double speed, double stopDis){
        currentPath=0;
        List<CurvePoint> allPointsF= new ArrayList<>(allPoints);
        double deltY=allPoints.get(allPoints.size()-1).y-allPoints.get(allPoints.size()-2).y;
        double deltX=allPoints.get(allPoints.size()-1).x-allPoints.get(allPoints.size()-2).x;

        allPointsF.remove(allPointsF.size()-1);
        allPointsF.add(new CurvePoint(allPoints.get(allPoints.size()-1).x+deltX, allPoints.get(allPoints.size()-1).y+deltY, allPoints.get(allPoints.size()-1).heading));
        while(Math.abs( distanceToPoint(getX(), getY(),allPoints.get(allPoints.size()-1).x, allPoints.get(allPoints.size()-1).y) )>stopDis && linearOpMode.opModeIsActive()){
            

            followCurve(allPointsF, followDistance, speed);
            //System.out.println(allPointsF.get(allPointsF.size()-1).x + " "+ allPointsF.get(allPointsF.size()-1).y);

            //allPointsF.clear();
/*
            linearOpMode.telemetry.addData("lf", leftFront.getPower());
            linearOpMode.telemetry.addData("lb", leftBack.getPower());
            linearOpMode.telemetry.addData("rb", rightBack.getPower());
            linearOpMode.telemetry.addData("rf", rightFront.getPower());
            linearOpMode.telemetry.addData("distance", distanceToPoint(getX(), getY(),allPoints.get(allPoints.size()-1).x, allPoints.get(allPoints.size()-1).y));
            linearOpMode.telemetry.addData("Position", ("("+round1000(getX())+", "+round1000( getY() ) + ", " + round1000(getHeading())+")"));
            linearOpMode.telemetry.update();

 */
            try{
                Thread.sleep(10);
            }
            catch(Exception e){}
        }

        for(int i=0; i<driveTrain.length; i++){
            driveTrain[i].setPower(0);
        }
    }
    public static double[] motorPower(double x, double y, double turn){
        double[] powers=new double[4];
        //double power=Math.hypot(x,y);
        double angle=Math.atan2(y,x);

        //fl, bl, br, fr
        powers[0]=-1* Math.sin(angle+Math.PI/4)+turn;
        powers[1]=-1*Math.cos(angle+Math.PI/4)+turn;
        powers[2]=-1*Math.sin(angle+Math.PI/4)-turn;
        powers[3]=-1*Math.cos(angle+Math.PI/4)-turn;

        return powers;
    }
    double kp=1/Math.PI/2.0;
    double ki=0;
    double kd=0;

    public void turn(double endAngle){
        double integral = 0;
        double newTime = runtime.seconds();
        double oldTime = 0;
        double error = angleWrap(imu.getAngularOrientation().firstAngle - endAngle);
        oldTime=newTime;
        newTime = runtime.seconds();
        double timeDiff = newTime - oldTime;

        if(timeDiff==0){
            timeDiff+=.00001;
        }

        double oldError = error;
        error = angleWrap(imu.getAngularOrientation().firstAngle - endAngle);

        double errorDif = angleWrap(error - oldError);

        double slope = errorDif/timeDiff;

        integral = integral + timeDiff * ((oldError + error)/2.0);

        double output = kp* error + kd* slope + ki * integral +.3;

        double[] powers= drive(0,0,output);

        for(int i=0; i<4; i++){
            driveTrain[i].setPower(powers[i]);
        }

        try{
            Thread.sleep(100); //sleep 1 millisec

        }
        catch(Exception e){}

        for(int i=0; i<4; i++){
            driveTrain[i].setPower(0);
        }
    }
    public static void setEndPosition(CurvePoint end){
        endX=end.x;
        endY=end.y;
        endAngle=end.heading;
    }

}
