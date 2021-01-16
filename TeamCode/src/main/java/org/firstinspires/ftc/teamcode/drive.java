package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="drive", group="f")
public class drive extends LinearOpMode{
    robot boWei = new robot();

    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        //float current = 0;
        float currentVex = 0;
        waitForStart();
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

        double rate=1;
        boolean bIsPressed=false;
        boolean collecting=false;

        boolean fieldOriented= false;

        boolean prevRStick=false;
        boolean prevLStick=false;
        while(boWei.linearOpMode.opModeIsActive()){
            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;
            
            boolean lsb=gamepad1.left_stick_button;
            boolean rsb=gamepad1.right_stick_button;

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
            if (gamepad2.dpad_up && !dUpHeld){
                rate++;
                if (rate > 6){
                    rate=6;
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
                if(rate>6){
                    rate=6;
                }

                dRightHeld=true;
            }
            if(!gamepad2.dpad_right){
                dRightHeld=false;
            }


            //collection
            if (gamepad2.left_bumper){
                boWei.spin.setPower(0.8);
            }
            else if (gamepad2.right_bumper){
                boWei.spin.setPower(-0.8);
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
                    isMoving = true;
                }
                aIsPressed = true;
            }
            if (!gamepad2.a){
                aIsPressed = false;
            }

            if(isMoving){
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
                    boWei.upwards.setPower(-0.8);
                    collecting=true;
                }
                bIsPressed=true;
            }
            if(!gamepad2.b){
                bIsPressed=false;
            }

            if (!lsb &&!rsb && prevLStick){
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
                //leftPressed = true;
            }

            if(lsb && rsb &&!leftPressed && !rightPressed){
                fieldOriented= !fieldOriented;
                if(fieldOriented){
                    telemetry.speak("field oriented, activated");
                }
                else{
                    telemetry.speak("robot oriented, activated");
                }
                leftPressed=true;
                rightPressed=true;
            }

            if(!lsb){
                leftPressed = false;
            }
            if(!rsb){
                rightPressed=false;
            }


            double lf;
            double lb;
            double rf;
            double rb;

            double ratio;

            if(fieldOriented){
                double magnitude= Math.hypot(ly, lx);
                double angle= boWei.angleWrap(Math.atan2(ly, -lx)-Math.PI/4 + boWei.imu.getAngularOrientation().firstAngle);

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
                ratio = magnitude / max;
                if (max == 0) {
                    ratio=0;
                }
            }


            telemetry.addData("LF", lf);
            telemetry.addData("LB", lb);
            telemetry.addData("RF", rf);
            telemetry.addData("RB", rb);
            telemetry.addData("rad/s", boWei.launch.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("tick/s", boWei.launch.getVelocity());
            telemetry.addData("rate", rate);
            telemetry.addData("Field oriented: ", fieldOriented);
            telemetry.update();

            boWei.leftFront.setPower(lf * ratio * toggle);
            boWei.leftBack.setPower(lb * ratio * toggle);
            boWei.rightFront.setPower(rf * ratio * toggle);
            boWei.rightBack.setPower(rb * ratio * toggle);

            prevLStick=lsb;
            prevRStick=rsb;
        }

    }
}
