package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import static org.firstinspires.ftc.teamcode.utilities.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

@TeleOp(name="drive2", group="f")
@Disabled
public class drive2 extends LinearOpMode{
    robot2Wheel boWei = new robot2Wheel();

    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        //float current = 0;
        //float currentVex = 0;
        //boWei.location.setPosition(robot.endX, robot.endY, robot.endAngle);
        //boWei.location.setPosition(0, 0, 0);

        waitForStart();


        double integral =0;

        double angleDiff=0;

        double prevAngleDiff=angleDiff;


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

        boolean distanceBasedShoot=false;
        double rate=1;

        double oldAngle = 0;
        boolean bIsPressed=false;
        boolean collecting=false;

        boolean fieldOriented= false;

        boolean prevRStick=false;
        boolean prevLStick=false;

        boolean aimLock=false;




        //FtcDashboard dashboard = FtcDashboard.getInstance();

        while(opModeIsActive()){


            boolean lsb=gamepad1.left_stick_button;
            boolean rsb=gamepad1.right_stick_button;

            boolean lbump= gamepad1.left_bumper;


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
                boWei.reverseCollect();
            }
            else if (gamepad2.right_bumper){
                boWei.collect();
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
                    boWei.upward();
                    collecting=true;
                }
                bIsPressed=true;
            }
            if(!gamepad2.b){
                bIsPressed=false;
            }



            if(!lsb){
                leftPressed = false;
            }
            if(!rsb){
                rightPressed=false;
            }




            //telemetry.addData("LF", lf);
            //telemetry.addData("LB", lb);
            //telemetry.addData("RF", rf);
            //telemetry.addData("RB", rb);

//            telemetry.addData("left", -boWei.leftOdo.getCurrentPosition());
//            telemetry.addData("right", boWei.rightOdo.getCurrentPosition());
//            telemetry.addData("horizontal", boWei.horizontalOdo.getCurrentPosition());

            telemetry.addData("rad/s", boWei.launch.getVelocity(AngleUnit.RADIANS));
            telemetry.addData("tick/s", boWei.launch.getVelocity());
            telemetry.addData("rate", rate);
            telemetry.addData("Field oriented: ", fieldOriented);



            telemetry.update();


            TelemetryPacket packet = new TelemetryPacket();
            //packet.fieldOverlay().setFill("Green").setStrokeWidth(1).setStroke("goldenrod").fillCircle(boWei.getX(), boWei.getY(), 5);
            //dashboard.sendTelemetryPacket(packet);



//            boWei.leftFront.setPower(lf * ratio * toggle);
//            boWei.leftBack.setPower(lb * ratio * toggle);
//            boWei.rightFront.setPower(rf * ratio * toggle);
//            boWei.rightBack.setPower(rb * ratio * toggle);


        }

    }
}
