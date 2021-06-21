package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.round1000;

@Autonomous(name = "powerRedLeft", group = "hmm")
public class powerRedLeft extends LinearOpMode {
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap,this);
        boWei.initOpenCV();
        int ringsShot = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int cases;
        boolean counter = false;
        boWei.location.setPosition(-62, -16, Math.PI);
        List<CurvePoint> goLeft = new ArrayList<>();
        //List<CurvePoint> goMid = new ArrayList<>();
        List<CurvePoint> goGoal = new ArrayList<>();
        List<CurvePoint> toPark = new ArrayList<>();

        goLeft.add(new CurvePoint(boWei.getPosition()));
        //goLeft.add(new CurvePoint(-36,-16, Math.PI));
        goLeft.add(new CurvePoint(-10,-18,-2.74));
        /*
        goMid.add(new CurvePoint(boWei.getPosition()));
        goMid.add(new CurvePoint(-1.031,-6,-2.842));

        goRight.add(new CurvePoint(boWei.getPosition()));
        goRight.add(new CurvePoint(-1.807,-5.686,-2.938));

         */
        goGoal.add(new CurvePoint(12,-30, Math.PI));

        //toPark.add(new CurvePoint(-12, -24, Math.PI));
        toPark.add(new CurvePoint(4,-12, Math.PI));
        toPark.add(new CurvePoint(8, -24, Math.PI));
        waitForStart();


        sleep(3000);

        cases=boWei.pipeline.getRings();
        telemetry.addData("rings0", cases);
        //boWei.grab();
        //boWei.goStiff();
        //boWei.grabber.setPosition(0.16);
        //boWei.autonUnflip();
        //boWei.flip();
        //boWei.flippyFlip.setPosition(.88);
        telemetry.update();
        //telemetry.addData("Position", boWei.getPosition());
        //telemetry.addData("Position", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
        if (cases == 1){
            goGoal.add(new CurvePoint(40, -18, Math.PI)); // 40, 15
        }
        else if (cases == 0){
            goGoal.add(new CurvePoint(24, -48, Math.PI)); //12,-40 could change angle to Math.PI/2 facing sideways
        }
        else if (cases == 4){
            goGoal.add(new CurvePoint(58,-43, Math.PI)); //15,-48
        }
        //40, 15

        //telemetry.addData("position",boWei.getPosition());
        boWei.launch.setVelocity(8.0, AngleUnit.RADIANS);
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("Green").setStrokeWidth(1).setStroke("goldenrod").fillCircle(boWei.getX(), boWei.getY(), 5);
        //packet.put("velocity", boWei.launch.getVelocity(AngleUnit.RADIANS));
        //packet.put("rate", rate);
        //packet.put("want rate", 8.6);
        dashboard.sendTelemetryPacket(packet);

        boWei.followCurveSync(goLeft, 8, 0.4,5);

        int a = 1;
        telemetry.addData("stop:", a);

        boWei.turnTo(-2.6 ,0.3);
        while (ringsShot == 0 && opModeIsActive()){
            telemetry.addData("rings1", ringsShot);
            telemetry.addData("ringsShot1",ringsShot);
            telemetry.addData("Position0", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
            //telemetry.addData("position", boWei.getPosition());
            telemetry.update();
            boolean hasRing = false;
            if (boWei.name.alpha() > 200){
                hasRing = true;
            }
            if (!hasRing && counter){
                ringsShot++;
            }
            counter = hasRing;

            boWei.collect();
            boWei.upward();
        }
        boWei.spin.setPower(0);
        boWei.upwards.setPower(0);
        boWei.turnTo(-2.7, 0.3);
        sleep(1500);
        while (ringsShot == 1 && opModeIsActive()){
            telemetry.addData("Position1", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
            telemetry.addData("ringsShot2",ringsShot);
            telemetry.update();
            boolean hasRing = false;
            if (boWei.name.alpha() > 200){
                hasRing = true;
            }
            if (!hasRing && counter){
                ringsShot++;
            }
            counter = hasRing;
            boWei.upward();
            boWei.collect();
        }
        boWei.upwards.setPower(0);
        boWei.spin.setPower(0);
        boWei.turnTo(-2.8, 0.3);
        sleep(1500);
        while (ringsShot == 2 && opModeIsActive()){
            telemetry.addData("Position2", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
            telemetry.addData("ringsShot3",ringsShot);
            telemetry.update();
            boolean hasRing = false;
            if (boWei.name.alpha() > 200){
                hasRing = true;
            }
            if (!hasRing && counter){
                ringsShot++;
            }
            counter = hasRing;
            boWei.upward();
            boWei.collect();
        }

        boWei.upwards.setPower(0);
        boWei.spin.setPower(0);
        boWei.launch.setPower(0);

        telemetry.addData("ringsShot4",ringsShot);
        telemetry.addData("Position3", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
        telemetry.update();
        //boWei.followCurveSync(goGoal,8, 0.5, 8);
        //boWei.autonUnflip();
        sleep(500);
        //boWei.release();
        sleep(500);
        //boWei.flip();
        toPark.add(0, boWei.getPosition());
        boWei.followCurveSync(toPark,8,0.5,8);
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        //robot.setEndPosition(boWei.getPosition());
    }

}
