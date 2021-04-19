package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.angleWrap;

@Autonomous(name="bot", group = "L")
public class f extends LinearOpMode {
    robot boWei = new robot();


    @Override
    public void runOpMode(){
        boWei.init(hardwareMap,this);
        boWei.initOpenCV();

        int cases = boWei.pipeline.getRings();
        boWei.location.setPosition(-62, -24, Math.PI);

        List<CurvePoint> toShoot= new ArrayList<>();
        toShoot.add(new CurvePoint(boWei.getPosition()));
        toShoot.add(new CurvePoint(12, -24, Math.PI));
        //toShoot.add(new CurvePoint(-25, -40, Math.PI));

        List<CurvePoint> toGoal = new ArrayList<>();


        List<CurvePoint> toPark = new ArrayList<>();
        toPark.add(new CurvePoint(0, -24, Math.PI));


        waitForStart();
        sleep(500);
        cases=boWei.pipeline.getRings();

        if (cases == 1){
            toGoal.add(new CurvePoint(36, -36, Math.PI));
        }
        else if (cases == 0){
            toGoal.add(new CurvePoint(12, -60, Math.PI));
        }
        else if (cases == 4){
            toGoal.add(new CurvePoint(60,-60, Math.PI));
        }

        boWei.followCurveSync(toShoot, 8, .8, 4);

        double angleDiff= angleWrap(boWei.getHeading()+Math.PI- Math.atan2(robot.redGoal.y-boWei.getY(), robot.redGoal.x-boWei.getX()));

        while(Math.abs(angleDiff)>.015 && opModeIsActive()){
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


        boWei.launch.setVelocity(boWei.getRate(), AngleUnit.RADIANS);//will have to change getRate later
        boWei.collect();
        boWei.upward();
        sleep(5000);
        boWei.launch.setPower(0);
        boWei.spin.setPower(0);
        boWei.upwards.setPower(0);

        toGoal.add(0, boWei.getPosition());
        boWei.followCurveSync(toGoal, 8, .8, 4);


        toPark.add(0, boWei.getPosition());
        boWei.followCurveSync(toPark, 8, .8, 4);

    }

}
