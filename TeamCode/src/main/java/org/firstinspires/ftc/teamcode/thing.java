package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="thingy", group = "f")
public class thing extends LinearOpMode {
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        boolean counter = false;

        int ringsShot = 0;
        waitForStart();
        while (opModeIsActive()){
            sleep(500);
            /*
            boolean hasRing = boWei.hasRing(boWei.thanks.getDistance(DistanceUnit.CM));
            if (!hasRing && counter) {
                ringsShot++;
            }
            counter = hasRing;
            if (ringsShot == 0) {
                boWei.launch.setVelocity(8.0, AngleUnit.RADIANS);
                boWei.spin.setPower(-0.8);
                boWei.upwards.setPower(-0.8);
            }
            //sleep(5000);
            //boWei.launch.setVelocity(0);
            //boWei.spin.setPower(0);
            //boWei.upwards.setPower(0);

             */
            if (ringsShot > 0){
                boWei.launch.setVelocity(0);
                boWei.spin.setPower(0);
                boWei.upwards.setPower(0);
            }
            telemetry.addData("rings:", ringsShot);
            //telemetry.addData("hasRing", hasRing);
            //telemetry.addData("Distance", boWei.thanks.getDistance(DistanceUnit.CM));
            telemetry.update();

        }
    }

}

