package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.angleWrap;

@Autonomous(name="redPure", group="pure")
public class redPure extends LinearOpMode {
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        boWei.initOpenCV();

        List<CurvePoint> toShoot= new ArrayList<>();
        boWei.location.setPosition(-65, -18, Math.PI);
        toShoot.add(new CurvePoint(boWei.getPosition()));
        toShoot.add(new CurvePoint(-25, -18, Math.PI));

        List<CurvePoint> toPark = new ArrayList<>();
        toPark.add(new CurvePoint(0, -24, Math.PI));

        waitForStart();

        double rate=.0136904762 * (Math.hypot(robot.redGoal.x-boWei.getX(), robot.redGoal.y-boWei.getY()))+3.542857143;

        boWei.launch.setVelocity(rate, AngleUnit.RADIANS);
        boWei.followCurveSync(toShoot, 12, .5, 2);


        rate=.0136904762 * (Math.hypot(robot.redGoal.x-boWei.getX(), robot.redGoal.y-boWei.getY()) )+3.542857143;

        boWei.launch.setVelocity(rate, AngleUnit.RADIANS);

        double angleDiff= angleWrap(boWei.getHeading()+Math.PI- Math.atan2(robot.redGoal.y-boWei.getY(), robot.redGoal.x-boWei.getX()));

        while(Math.abs(angleDiff)>.1 && opModeIsActive()){
            double rx= angleDiff/Math.abs(angleDiff)*( Math.abs(angleDiff)/6.0 + .3);
            double[] powers= boWei.drive(0, 0, rx);
            for(int i=0; i<boWei.driveTrain.length; i++){
                boWei.driveTrain[i].setPower(powers[i]);
            }
            angleDiff= angleWrap(boWei.getHeading()+Math.PI- Math.atan2(robot.redGoal.y-boWei.getY(), robot.redGoal.x-boWei.getX()));
        }

        for(int i=0; i<boWei.driveTrain.length; i++){
            boWei.driveTrain[i].setPower(0);
        }

        boWei.upwards.setPower(-.8);
        boWei.spin.setPower(.8);

        sleep(5300);

        boWei.launch.setVelocity(0);
        boWei.upwards.setPower(0);
        boWei.spin.setPower(0);

        toPark.add(0, boWei.getPosition());

        boWei.followCurveSync(toPark, 8, .8, 4);


    }
}