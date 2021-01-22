package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.utilities.*;
public class Position extends Thread{
    //position stuff
    double x,y,heading;

    //pass in the robot's odometry encoders
    DcMotor leftOdo, rightOdo, horizontalOdo;

    //also pass in robot's imu
    BNO055IMU imu;
    LinearOpMode opMode;

    //constants to convert encoders to distance moved
    final double encoderToInch=1; //will need to tune
    final double horizEncoderToInch=1; //will need to tune

    //current encoder position/ angle position
    double positionLeft;
    double positionRight;
    double positionHoriz;
    double angle;

    //previous positions
    double previousLeft;
    double previousRight;
    double previousHoriz;
    double previousAngle;

    //use inches for x and y positions
    public Position(double x, double y, double heading, DcMotor leftOdo, DcMotor rightOdo, DcMotor horizontalOdo, BNO055IMU imu, LinearOpMode opMode){
        //set all the stuff above equal to parameters obviously
        this.x=x;
        this.y=y;
        this.heading=heading;
        this.imu=imu;
        this.leftOdo=leftOdo;
        this.rightOdo=rightOdo;
        this.horizontalOdo=horizontalOdo;

        this.opMode=opMode;

        //set an initial position
        positionLeft=this.leftOdo.getCurrentPosition();
        positionRight=this.rightOdo.getCurrentPosition();
        positionHoriz=this.horizontalOdo.getCurrentPosition();
        angle=imu.getAngularOrientation().firstAngle;
    }

    @Override
    public void run(){
        while(!opMode.isStopRequested()){
            //transfer positions to previous positions
            previousLeft=positionLeft;
            previousRight=positionRight;
            previousHoriz=positionHoriz;
            previousAngle=angle;

            //get current positions
            positionLeft=this.leftOdo.getCurrentPosition();
            positionRight=this.rightOdo.getCurrentPosition();
            positionHoriz=this.horizontalOdo.getCurrentPosition();
            angle=imu.getAngularOrientation().firstAngle;

            //find the change in positions
            double changeLeft=positionLeft-previousLeft;
            double changeRight=positionRight-previousRight;
            double changeHoriz=positionHoriz-previousHoriz;
            double changeAngle=angleWrap(angle-previousAngle);

            //the amount the bot moved is the average of the left and right
            double changeBot=(changeLeft+changeRight)/2.0;

            //find the change in position of the robot in global coordinates (X,Y coordinate field)
            double deltaX=encoderToInch* changeBot*Math.cos(heading+changeAngle/2.0) +horizEncoderToInch* changeHoriz*Math.cos(heading+changeAngle/2.0);
            double deltaY=encoderToInch* changeBot*Math.sin(heading+changeAngle/2.0) -horizEncoderToInch* changeHoriz*Math.sin(heading+changeAngle/2.0);

            //add these changes to the xy etc
            heading+=changeAngle;
            x+=deltaX;
            y+=deltaY;

            try{
                Thread.sleep(10);
            }
            catch (InterruptedException e){}


        }

    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void setPosition(double x, double y, double heading){
        this.x=x;
        this.y=y;
        this.heading=heading;
    }
}
