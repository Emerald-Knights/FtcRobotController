package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.utilities.angleWrap;

@TeleOp(name="drive simple", group="f")
@Disabled
public class driveSimple extends LinearOpMode {
    robot boWei=new robot();
    boolean fieldOriented=false;
    boolean lbumpPressed=false;
    @Override
    public void runOpMode(){

        boWei.init(hardwareMap, this);
        waitForStart();
        while(opModeIsActive()){
            double lf;
            double lb;
            double rf;
            double rb;

            double ratio;

            boolean lsb=gamepad1.left_stick_button;
            boolean rsb=gamepad1.right_stick_button;

            boolean lbump= gamepad1.left_bumper;
            double lx=gamepad1.left_stick_x;
            double ly=-gamepad1.left_stick_y;
            double rx=gamepad1.right_stick_x;


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
            boWei.leftBack.setPower(lb * ratio);
            boWei.leftFront.setPower(lf * ratio);
            boWei.rightBack.setPower(rb * ratio);
            boWei.rightFront.setPower(rf * ratio);

        }


    }
}
