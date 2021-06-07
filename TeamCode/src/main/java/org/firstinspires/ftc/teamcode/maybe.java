package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.utilities.round1000;

@Autonomous(name = "powerMax", group = "blub")
public class maybe extends LinearOpMode {
    robot boWei = new robot();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap,this);
        boWei.initOpenCV();
        int ringsShot = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int cases;
        boolean counter = false;
        boWei.location.setPosition(-62, -40, Math.PI);
        List<CurvePoint> goLeft = new ArrayList<>();
        //List<CurvePoint> goMid = new ArrayList<>();
        //List<CurvePoint> goRight = new ArrayList<>();
        List<CurvePoint> highGoal = new ArrayList<>();
        List<CurvePoint> goGoal = new ArrayList<>();
        List<CurvePoint> toPark = new ArrayList<>();
        List<CurvePoint> goShoot = new ArrayList<>();
        goLeft.add(new CurvePoint(boWei.getPosition()));
        //goLeft.add(new CurvePoint(-37,-24, Math.PI)); add to avoid hitting ring stack
        goLeft.add(new CurvePoint(-1,-20,-2.74));

        goShoot.add(new CurvePoint(0,-18,Math.PI));
        goShoot.add(new CurvePoint(0, -36, Math.PI));
        /*
        goMid.add(new CurvePoint(boWei.getPosition()));
        goMid.add(new CurvePoint(-1.031,-6,-2.842));

        goRight.add(new CurvePoint(boWei.getPosition()));
        goRight.add(new CurvePoint(-1.807,-5.686,-2.938));

         */
        goGoal.add(new CurvePoint(12,-30, Math.PI));

        //toPark.add(new CurvePoint(4,-12, Math.PI));
        toPark.add(new CurvePoint(8, -24, Math.PI));
        waitForStart();


        sleep(3000);

        cases=boWei.pipeline.getRings();
        telemetry.addData("rings0", cases);
        boWei.grabber.setPosition(0.16);
        boWei.flippyFlip.setPosition(.88);
        //telemetry.addData("Position", boWei.getPosition());
        //telemetry.addData("Position", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
        telemetry.update();
        if (cases == 1){
            goGoal.add(new CurvePoint(20, -40, Math.PI)); // 40, 15
        }
        else if (cases == 0){
            goGoal.add(new CurvePoint(24, -48, Math.PI)); //12,-40 could change angle to Math.PI/2 facing sideways
        }
        else if (cases == 4){
            goGoal.add(new CurvePoint(54,-44, Math.PI)); //15,-48
            telemetry.addData("4",4);
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

        if (boWei.getHeading() > -2.66){
            while (Math.abs(2.66 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
                boWei.rightBack.setPower(-0.3);
                boWei.rightFront.setPower(-0.3);
                boWei.leftBack.setPower(0.3);
                boWei.leftFront.setPower(0.3);
            }
            boWei.rightBack.setPower(0);
            boWei.rightFront.setPower(0);
            boWei.leftBack.setPower(0);
            boWei.leftFront.setPower(0);
        }
        else if (boWei.getHeading() < -2.66){
            while (Math.abs(2.66 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
                boWei.rightBack.setPower(0.3);
                boWei.rightFront.setPower(0.3);
                boWei.leftBack.setPower(-0.3);
                boWei.leftFront.setPower(-0.3);
            }
            boWei.rightBack.setPower(0);
            boWei.rightFront.setPower(0);
            boWei.leftBack.setPower(0);
            boWei.leftFront.setPower(0);
        }
        timer.reset();
        while (ringsShot == 0 && opModeIsActive() && timer.seconds() < 4){
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
        while (Math.abs(2.72 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
            boWei.rightBack.setPower(-0.3);
            boWei.rightFront.setPower(-0.3);
            boWei.leftBack.setPower(0.3);
            boWei.leftFront.setPower(0.3);
        }
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        //boWei.moveToPosition(boWei.getX(),boWei.getY(),0.4,-2.842);
        sleep(1500);
        timer.reset();
        while (ringsShot == 1 && opModeIsActive() && timer.seconds() < 4){
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
        while (Math.abs(2.81 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
            boWei.rightBack.setPower(-0.3);
            boWei.rightFront.setPower(-0.3);
            boWei.leftBack.setPower(0.3);
            boWei.leftFront.setPower(0.3);
        }
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        sleep(1500);
        timer.reset();
        while (ringsShot == 2 && opModeIsActive() && timer.seconds() < 4){
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
        boWei.followCurveSync(goGoal,8, 0.45, 10);
        boWei.grabber.setPosition(0.6);
        sleep(500);
        boWei.flippyFlip.setPosition(0.5);
        sleep(1000);
        highGoal.add(new CurvePoint(boWei.getX(), -36,Math.PI));
        highGoal.add(new CurvePoint(-20, -36,Math.PI));
        sleep(500);
        boWei.followCurveSync(highGoal,8,0.5,5);

        boWei.collect();
        sleep(2000);
        boWei.launch.setVelocity(8.4, AngleUnit.RADIANS);
        boWei.followCurveSync(goShoot,8,0.5,0.8);
        while (Math.abs(2.81 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
            boWei.rightBack.setPower(-0.3);
            boWei.rightFront.setPower(-0.3);
            boWei.leftBack.setPower(0.3);
            boWei.leftFront.setPower(0.3);
        }
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        sleep(400);
        boWei.upward();
        sleep(4000);
        boWei.spin.setPower(0);
        boWei.launch.setPower(0);
        boWei.upwards.setPower(0);
        toPark.add(0, boWei.getPosition());
        boWei.followCurveSync(toPark,8,0.5,5);
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        //robot.setEndPosition(boWei.getPosition());
    }

}
