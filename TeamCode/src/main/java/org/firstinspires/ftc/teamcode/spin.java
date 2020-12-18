package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
@TeleOp(name="drive", group="f")
public class spin extends LinearOpMode{
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        //float current = 0;
        float currentVex = 0;
        waitForStart();
        while(boWei.linearOpMode.opModeIsActive()){
            boolean x1= gamepad1.x;
            boolean y1= gamepad1.y;

            double lx=-gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=-gamepad1.right_stick_x;

            if (Math.abs(lx) <= 0.15) {
                lx = 0;
            }

            if (Math.abs(ly) <= 0.15) {
                ly = 0;
            }

            if (Math.abs(rx) <= 0.15) {
                rx = 0;
            }

            if(gamepad1.dpad_up){
                //boWei.leftLift.setPosition(.46); //powershot
                boWei.rightLift.setPosition(.54);
            }
            if(gamepad1.dpad_down){
                //boWei.leftLift.setPosition(.43); //third thing
                boWei.rightLift.setPosition(.75);
            }

            double lf = ly + rx + lx;
            double lb = ly - rx + lx;
            double rf = ly - rx - lx;
            double rb = ly + rx - lx;


            double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
            double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
            double ratio = magnitude / max;
            if (max == 0) {
                ratio=0;
            }
            telemetry.addData("LF", lf);
            telemetry.addData("LB", lb);
            telemetry.addData("RF", rf);
            telemetry.addData("RB", rb);
            telemetry.addData("ratio", ratio);
            telemetry.addData("position", boWei.rightLift.getPosition());
            telemetry.update();
            boWei.leftFront.setPower(-lf * ratio);
            boWei.leftBack.setPower(lb * ratio);
            boWei.rightFront.setPower(-rf * ratio);
            boWei.rightBack.setPower(rb * ratio);


            if(gamepad1.right_bumper){
                boWei.spin.setPower(-0.8);
            }
            else if(gamepad1.left_bumper){
                boWei.spin.setPower(0.8);
            }
            else{
                boWei.spin.setPower(0);
            }

            if (gamepad1.a){
                boWei.launch.setPower(0.8);
            }
            else if (gamepad1.b){
                boWei.launch.setPower(-0.8);
            }
            else{
                boWei.launch.setPower(0);
            }

        }

    }
}
