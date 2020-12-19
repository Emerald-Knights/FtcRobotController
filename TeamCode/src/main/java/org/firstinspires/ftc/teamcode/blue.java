package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

@Autonomous(name="auton", group = "sus")
public class blue extends LinearOpMode {
    robot boWei= new robot();

    enum Cases {
        one, two, three, four
    }

    Cases c=Cases.three;

    @Override
    public void runOpMode() {
        boWei.init(hardwareMap, this);
        waitForStart();

        //start ur code here
        //case 1 (closest)
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
            boWei.move(2700, 1);
            //boWei.move(950, -1);
        } else if (c == Cases.four) {
            boWei.turning(Math.PI);
            boWei.turning(Math.PI / 2.0);
        }


    }
}
//yoink
//hi blah blah blah what is this