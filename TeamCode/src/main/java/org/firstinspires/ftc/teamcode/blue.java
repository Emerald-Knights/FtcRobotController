package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="auton", group = "sus")
public class blue extends LinearOpMode {
    robot boWei= new robot();

    enum Cases {
        one, two, three, four, five
    }

    Cases c=Cases.five;
    Recognition max;

    @Override
    public void runOpMode() {
        boWei.init(hardwareMap, this);

        //boWei.initTF();
        //ElapsedTime runtime= new ElapsedTime();

        waitForStart();
        //runtime.reset();

        boWei.spin.setPower(0.8);
        sleep(500);
        boWei.upwards.setPower(-0.8);
        boWei.launch.setPower(.8);

/*
        while(runtime.seconds()<5){
            List<Recognition> list = boWei.tfod.getUpdatedRecognitions();
            if (list != null) {
                max = list.get(0);
                for (int i = 0; i < list.size(); i++){
                    if (max.getConfidence() < list.get(i).getConfidence()){
                        max = list.get(i);
                    }
                }

            }

            if(max!=null){
                if (max.getLabel().equals("Quad")){
                    c = Cases.two;
                }
                else if(max.getLabel().equals("Single")){
                    c = Cases.three;
                }
            }
            else{
                c = Cases.one;
            }
        }




 */

        //start ur code here
        //case 1 (closest)
        /*
        if (c == Cases.one) {
            boWei.move(1558, 1);
            boWei.move(200, -1);
            boWei.strafe(420, -1);
            boWei.move(250, 1);
        }
        //case 2 (we are going to need to turn on this one)
        else if (c == Cases.two) {
            boWei.move(1857, 1);
            boWei.turning(-Math.PI / 4.0);
            boWei.move(350, 1);
            boWei.move(300, -1);
            boWei.turning(-Math.PI);
            boWei.move(150, 1);
        }
        //case 3 (furthest)
        else if (c == Cases.three) {
            boWei.move(2500, 1);
            //boWei.move(950, -1);
        } else if (c == Cases.four) {
            boWei.turning(Math.PI);
            boWei.turning(Math.PI / 2.0);
        }
        else if(c== Cases.five){

            boWei.move(2400, -1);

        }

         */
        boWei.move(2400, -1);
        sleep(500);
        boWei.spin.setPower(0);
        boWei.upwards.setPower(0);
        boWei.launch.setPower(0);
    }
}
//yoink
//hi blah blah blah what is this