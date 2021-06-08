package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import static org.firstinspires.ftc.teamcode.utilities.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Point;

@TeleOp(name="drive", group="f")
public class drive extends LinearOpMode{
    robot boWei = new robot();

    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        //float current = 0;
        //float currentVex = 0;
        //boWei.location.setPosition(robot.endX, robot.endY, robot.endAngle);
        //boWei.location.setPosition(0, -36, 0); 0,0,0
        boWei.location.setPosition(robot.endX, robot.endY, robot.endAngle);

        waitForStart();

        boWei.runtime.reset();

        double integral =0;
        double curTime=boWei.runtime.seconds();

        double angleDiff=0;

        double prevAngleDiff=angleDiff;

        double prevTime=curTime;


        boolean isMoving = false;
        boolean aIsPressed = false;
        boolean leftPressed = false;
        boolean rightPressed=false;
        boolean slowmodeActive = false;
        boolean dLeftHeld=false;
        boolean dRightHeld=false;
        boolean dUpHeld = false;
        boolean ddownHeld = false;
        double toggle = 1;
        boolean lbumpPressed=false;
        boolean yIsPressed=false;

        int ringsShot = 0;
        boolean counter = false;

        boolean distanceBasedShoot=true;
        double rate=1;

        double oldAngle = 0;
        boolean bIsPressed=false;
        boolean collecting=false;

        boolean fieldOriented= false;

        boolean prevRStick=false;
        boolean prevLStick=false;

        boolean aimLock=false;
        boolean y1Pressed = false;
        boolean x1Pressed = false;
        boolean x2Pressed = false;

        boolean flip=false;
        boolean grab = false;

        boolean xPressed = false;
        boolean moving = false;
        double maxPow=20;

        boolean a1Pressed=false;
        FtcDashboard dashboard = FtcDashboard.getInstance();


        boolean cheats=false;

        //boolean[] code = {false, false, false, false, false, false, false, false, false, false, false, false};
        int currentButton=0;

        while(opModeIsActive()){

            /*
            if(gamepad2.x && !xPressed){
                if (!moving){
                    boWei.moveToPosition(0,0,0,0);// find point later
                    moving = true;
                }
                else{
                    boWei.moveToPosition(boWei.getX(), boWei.getY(), 1, boWei.getHeading());
                }
                xPressed = true;
            }

             */

            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;
            
            boolean lsb=gamepad1.left_stick_button;
            boolean rsb=gamepad1.right_stick_button;

            boolean lbump= gamepad1.left_bumper;

            //deadzone
            if (Math.abs(lx) <= 0.15) {
                lx = 0;
            }

            if (Math.abs(ly) <= 0.15) {
                ly = 0;
            }

            if (Math.abs(rx) <= 0.15) {
                rx = 0;
            }
/*
            //shooter levels
            if(gamepad2.dpad_up){
                boWei.leftLift.setPosition(.46); //third thing
                boWei.rightLift.setPosition(.56);
            }
            if(gamepad2.dpad_down){
                boWei.leftLift.setPosition(.43); //powershot
                boWei.rightLift.setPosition(.53);
            }

 */
            if (gamepad2.x){
                boWei.location.setPosition(boWei.getX(), boWei.getY(), 0);
            }

            if (gamepad2.dpad_up && !dUpHeld){
                rate++;
                if (rate > maxPow){
                    rate=maxPow;
                }
                dUpHeld = true;
            }
            if (!gamepad2.dpad_up){
                dUpHeld = false;
            }
            if (gamepad2.dpad_down && !ddownHeld){
                rate--;
                if (rate < 0){
                    rate= 0;
                }

                ddownHeld = true;
            }
            if (!gamepad2.dpad_down){
                ddownHeld = false;
            }

            if(gamepad2.dpad_left && !dLeftHeld){
                rate-=.1;
                if(rate<0){
                    rate=0;
                }
                dLeftHeld=true;
            }
            if(!gamepad2.dpad_left){
                dLeftHeld=false;
            }
            if(gamepad2.dpad_right&& !dRightHeld){
                rate+=.1;
                if(rate>maxPow){
                    rate=maxPow;
                }

                dRightHeld=true;
            }
            if(!gamepad2.dpad_right){
                dRightHeld=false;
            }

            if (gamepad1.y && !y1Pressed){
                grab=!grab;
                if(grab){
                    boWei.grab();
                }
                else{
                    boWei.release();
                }

                y1Pressed = true;
            }
            if (!gamepad1.y){
                y1Pressed = false;
            }

            /*if (boWei.thanks.red() > boWei.thanks.blue()){
                counter = true;
            }
            else {
                counter = false;
            }

            boolean hasRing = boWei.hasRing(boWei.thanks.getDistance(DistanceUnit.CM));
            if (!hasRing && counter) {
                ringsShot++;
            }
            counter = hasRing;

             */

            /*
            boolean hasRing = false;
            if (boWei.hasRing(boWei.Dora.getDistance(DistanceUnit.CM))){ //|| boWei.hasRingC()){
                hasRing = true;
            }
            if (!hasRing && counter){
                ringsShot++;
            }
            counter = hasRing;
*/
            boolean hasRing = false;
            boolean temp = false;
            if (boWei.name.alpha() > 200){
                hasRing = true;
            }
            if (!hasRing && counter){
                //if (boWei.launch.getVelocity() < 6){
                    ringsShot++;
               // }
                //ringsShot++;
            }
            counter = hasRing;

            if (boWei.thanks.alpha() >6000){
                //counter = true;
                boWei.location.setX(36);
            }


           // telemetry.addData("grab:", grab);
            telemetry.addData("grabPos:", boWei.grabber.getPosition());
            //telemetry.addData("Distance:", boWei.Dora.getDistance(DistanceUnit.CM));
            //telemetry.addData("hasRing", boWei.hasRing(boWei.thanks.getDistance(DistanceUnit.CM)));
            telemetry.addData("rings", ringsShot);
            telemetry.addData("hasRing", counter);
            telemetry.addData("alpha", boWei.name.alpha());
            telemetry.addData("red",boWei.name.red());
            telemetry.addData("blue", boWei.name.blue());
            telemetry.addData("green",boWei.name.green());

            if (gamepad1.x && !x1Pressed){
                flip=!flip;
                if(flip){
                    boWei.flippyFlip.setPosition(0.5);
                }
                else{
                    boWei.flippyFlip.setPosition(0.93);

                }
                x1Pressed = true;
            }

            telemetry.addData("flippyFlip Position", boWei.flippyFlip.getPosition());
            telemetry.addData("fPort", boWei.flippyFlip.getPortNumber());
            telemetry.addData("GPort", boWei.grabber.getPortNumber());
            if (!gamepad1.x){
                x1Pressed = false;
            }
            //collection
            if (gamepad2.left_bumper){
                boWei.collect();
            }
            else if (gamepad2.right_bumper){
                boWei.reverseCollect();
            }
            else {
                boWei.spin.setPower(0);
            }

            //launcher
            if(gamepad2.a && !aIsPressed && !gamepad2.start){
                if (isMoving) {
                    telemetry.speak("Anthony is gay");
                    isMoving = false;
                }
                else if (!isMoving){
                    telemetry.speak("pew pew");
                    isMoving = true;
                }
                aIsPressed = true;
            }
            if (!gamepad2.a){
                aIsPressed = false;
            }
            if(gamepad2.y && !yIsPressed){
                distanceBasedShoot=!distanceBasedShoot;
                yIsPressed=true;
            }
            if(!gamepad2.y){
                yIsPressed=false;
            }

            if(isMoving){
                if(distanceBasedShoot){
                    //rate=.0136904762 * (Math.hypot(goal.x-boWei.getX(), goal.y-boWei.getY()))+3.542857143;
                    rate=boWei.getRate();

                }
                boWei.launch.setVelocity(rate, AngleUnit.RADIANS);
            }
            else{
                boWei.launch.setVelocity(0, AngleUnit.RADIANS);
            }

            //upwards
            if(gamepad2.b&&!bIsPressed){
                if(collecting){
                    boWei.upwards.setPower(0);
                    collecting=false;
                }
                else if(!collecting){
                    boWei.upward();
                    collecting=true;
                }
                bIsPressed=true;
            }
            if(!gamepad2.b){
                bIsPressed=false;
            }

            if (gamepad2.right_trigger > 0.3){
                boWei.upwards.setPower(0.8);
            }
            else if(gamepad2.left_trigger > 0.3){
                boWei.upwards.setPower(-0.8);
            }
            else{
                boWei.upwards.setPower(0);
            }

            if (lsb && !leftPressed){
                if (!slowmodeActive){
                    toggle = 0.5;
                    slowmodeActive = true;
                    telemetry.speak("Slowmode, activated");
                }
                else if (slowmodeActive) {
                    toggle = 1;
                    slowmodeActive = false;
                    telemetry.speak("Normal mode, activated");
                }
                leftPressed = true;
            }

            if(rsb && !rightPressed){
                aimLock= !aimLock;

                integral=0;

                rightPressed=true;
            }

            if(!lsb){
                leftPressed = false;
            }
            if(!rsb){
                rightPressed=false;
            }

            if(lbump && !lbumpPressed){
                fieldOriented=!fieldOriented;
                if(fieldOriented){
                    telemetry.speak("field oriented, activated");
                }
                else{
                    telemetry.speak("robot oriented, activated");
                }
                lbumpPressed=true;
            }
            if(!lbump){
                lbumpPressed=false;
            }

            if(aimLock){
                if(Math.abs(angleDiff)>.06){
                    double slope= (angleDiff-prevAngleDiff)/(curTime-prevTime);

                    integral+=(prevAngleDiff+angleDiff)/2.0*(curTime-prevTime);

                    //rx=angleDiff*boWei.kp + integral*boWei.ki + slope*boWei.kd+.3;
                    //rx= angleDiff/Math.abs(angleDiff)*( Math.abs(angleDiff)/6.0 + .3);
                    
                    rx=angleDiff/Math.abs(angleDiff)*( Math.abs(angleDiff)/12.0 + .3);
                }
                else{
                    rx=0;
                }
            }

            if(gamepad1.a && !a1Pressed){
                boWei.location.setPosition(0, -36, boWei.getHeading());
                a1Pressed=true;
            }
            if(!gamepad1.a){
                a1Pressed=false;
            }


            double lf;
            double lb;
            double rf;
            double rb;

            double ratio;

            if(fieldOriented){
                double magnitude= Math.hypot(ly, lx);
                double angle= angleWrap(Math.atan2(ly, -lx)-Math.PI/4 + boWei.getHeading());

                lf = (magnitude*Math.sin(angle) +rx)*.8;
                lb = (magnitude*Math.cos(angle) +rx)*.8;
                rf = (magnitude*Math.cos(angle) -rx)*.8;
                rb = (magnitude*Math.sin(angle) -rx)*.8;

                double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
                double mag=Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
                ratio = mag / max;
                if (max == 0) {
                    ratio=0;
                }
            }

            else{
                lf = ly + rx + lx;
                lb = ly + rx - lx;
                rf = ly - rx - lx;
                rb = ly - rx + lx;


                double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
                double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
                ratio = .8*magnitude / max;
                if (max == 0) {
                    ratio=0;
                }
            }

            double angle = oldAngle - boWei.getHeading();
            oldAngle = boWei.getHeading();


            //telemetry.addData("LF", lf);
            //telemetry.addData("LB", lb);
            //telemetry.addData("RF", rf);
            //telemetry.addData("RB", rb);
            telemetry.addData("left", -boWei.leftOdo.getCurrentPosition());
            telemetry.addData("right", boWei.rightOdo.getCurrentPosition());
            telemetry.addData("horizontal", boWei.horizontalOdo.getCurrentPosition());
            telemetry.addData("Diff:", (boWei.location.positionLeft + boWei.location.forwardEncoderToRadian *  angle) + (boWei.location.positionRight - boWei.location.forwardEncoderToRadian * -angle));
            telemetry.addData("Distance:", Math.hypot(boWei.redGoal.x-boWei.getX(), boWei.redGoal.y-boWei.getY()));
            telemetry.addData("Position", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
            //telemetry.addData("rings", ringsShot);
            telemetry.addData("distance based shoot?: ", distanceBasedShoot);
            telemetry.addData("rad/s", boWei.launch.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("tick/s", boWei.launch.getVelocity());
            telemetry.addData("rate", rate);
            telemetry.addData("Field oriented: ", fieldOriented);





            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setFill("Green").setStrokeWidth(1).setStroke("goldenrod").fillCircle(boWei.getX(), boWei.getY(), 5);
            packet.put("velocity", boWei.launch.getVelocity(AngleUnit.RADIANS));
            packet.put("rate", rate);
            //packet.put("want rate", 8.6);
            dashboard.sendTelemetryPacket(packet);



            boWei.leftFront.setPower(lf * ratio * toggle);
            boWei.leftBack.setPower(lb * ratio * toggle);
            boWei.rightFront.setPower(rf * ratio * toggle);
            boWei.rightBack.setPower(rb * ratio * toggle);

            prevLStick=lsb;
            prevRStick=rsb;

            prevAngleDiff=angleDiff;

            prevTime=curTime;

            angleDiff= boWei.getAngleDiff();
            curTime=boWei.runtime.seconds();
        }

    }
}
