package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.angleWrap;

@Autonomous(name="test", group="Lf")
@Disabled
public class test extends LinearOpMode {
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        boWei.initOpenCV();

        List<CurvePoint> toShoot= new ArrayList<>();
        boWei.location.setPosition(-62, -24, Math.PI);
        //boWei.location.setPosition(-62, -36, Math.PI);

        toShoot.add(new CurvePoint(boWei.getPosition()));
        toShoot.add(new CurvePoint(-25, -40, Math.PI));
        //toShoot.add(new CurvePoint(-25, -24, Math.PI));

        List<CurvePoint> toPark = new ArrayList<>();
        toPark.add(new CurvePoint(0, -24, Math.PI));

        waitForStart();

        double rate=.0136904762 * (Math.hypot(robot.redGoal.x-boWei.getX(), robot.redGoal.y-boWei.getY()))+3.542857143;

        boWei.launch.setVelocity(rate, AngleUnit.RADIANS);
        boWei.followCurveSync(toShoot, 12, .6, 5);

        sleep(500);

        rate=.0136904762 * (Math.hypot(robot.redGoal.x-boWei.getX(), robot.redGoal.y-boWei.getY()) )+3.542857143;

        boWei.launch.setVelocity(rate, AngleUnit.RADIANS);

        double angleDiff= angleWrap(boWei.getHeading()+Math.PI- Math.atan2(robot.redGoal.y-boWei.getY(), robot.redGoal.x-boWei.getX()));

        while(Math.abs(angleDiff)>.08 && opModeIsActive()){
            telemetry.addData("angle diff", angleDiff);
            telemetry.addData("position" , "("+boWei.getX()+","+boWei.getY()+")");
            telemetry.update();
            double rx= angleDiff/Math.abs(angleDiff)*( Math.abs(angleDiff)/12.0 + .3);
            double[] powers= boWei.drive(0, 0, rx);
            for(int i=0; i<boWei.driveTrain.length; i++){
                boWei.driveTrain[i].setPower(powers[i]);
            }
            angleDiff= angleWrap(boWei.getHeading()+Math.PI- Math.atan2(robot.redGoal.y-boWei.getY(), robot.redGoal.x-boWei.getX()));
        }

        for(int i=0; i<boWei.driveTrain.length; i++){
            boWei.driveTrain[i].setPower(0);
        }

        boWei.upwards.setPower(-.7);
        boWei.spin.setPower(.6);

        sleep(5300);

        boWei.launch.setVelocity(0);
        boWei.upwards.setPower(0);
        boWei.spin.setPower(0);

        toPark.add(0, boWei.getPosition());

        boWei.followCurveSync(toPark, 8, .8, 4);


    }
}
