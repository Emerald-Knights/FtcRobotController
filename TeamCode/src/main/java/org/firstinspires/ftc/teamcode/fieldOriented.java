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

@TeleOp(name="fieldDrive", group="f")
public class fieldOriented extends LinearOpMode{
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
        boolean slowmodeActive = false;
        double toggle = 1;

        boolean bIsPressed=false;
        boolean collecting=false;
        while(boWei.linearOpMode.opModeIsActive()){
            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;

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

            //shooter levels
            if(gamepad2.dpad_up){
                boWei.leftLift.setPosition(.46); //third thing
                boWei.rightLift.setPosition(.56);
            }
            if(gamepad2.dpad_down){
                boWei.leftLift.setPosition(.43); //powershot
                boWei.rightLift.setPosition(.53);
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
                    boWei.launch.setPower(0);
                    telemetry.speak("Anthony is gay");

                    isMoving = false;
                }
                else if (!isMoving){
                    boWei.launch.setVelocity(5, AngleUnit.RADIANS); //goes to ~6
                    isMoving = true;
                }
                aIsPressed = true;
            }
            if (!gamepad2.a){
                aIsPressed = false;
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

            if (gamepad1.left_stick_button && !leftPressed){
                if (!slowmodeActive){
                    toggle = 0.5;
                    slowmodeActive = true;
                    telemetry.speak("your ass");
                }
                else if (slowmodeActive) {
                    toggle = 1;
                    slowmodeActive = false;
                    telemetry.speak("what the fuck");
                }
                leftPressed = true;
            }
            if(!gamepad1.left_stick_button){
                leftPressed = false;
            }

            double magnitude= Math.hypot(ly, lx);
            double angle= boWei.angleWrap(Math.atan2(ly, -lx)-Math.PI/4 + boWei.imu.getAngularOrientation().firstAngle);

            double lf = (magnitude*Math.sin(angle) +rx)*.8;
            double lb = (magnitude*Math.cos(angle) +rx)*.8;
            double rf = (magnitude*Math.cos(angle) -rx)*.8;
            double rb = (magnitude*Math.sin(angle) -rx)*.8;

            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double mag=Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            double ratio = mag / max;
            if (max == 0) {
                ratio=0;
            }




            //double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            //double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            //double ratio = magnitude / max;
            //if (max == 0) {
            //    ratio=0;
            //}
            telemetry.addData("LF", lf);
            telemetry.addData("LB", lb);
            telemetry.addData("RF", rf);
            telemetry.addData("RB", rb);
            telemetry.addData("ratio", ratio);
            telemetry.addData("position", boWei.rightLift.getPosition());
            telemetry.addData("Button B:", gamepad2.b);
            telemetry.addData("rad/s", boWei.launch.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("angle", boWei.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            boWei.leftFront.setPower(lf * ratio * toggle);
            boWei.leftBack.setPower(lb * ratio * toggle);
            boWei.rightFront.setPower(rf * ratio * toggle);
            boWei.rightBack.setPower(rb * ratio * toggle);
        }

    }
}
