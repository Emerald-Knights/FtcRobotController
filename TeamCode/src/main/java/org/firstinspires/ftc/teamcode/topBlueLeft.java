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

@Autonomous(name = "topBlueLeft", group = "hmmm7")
public class topBlueLeft extends LinearOpMode {
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
        boWei.location.setPosition(-62, 40, Math.PI);
        List<CurvePoint> goMid = new ArrayList<>();
        //List<CurvePoint> goMid = new ArrayList<>();
        //List<CurvePoint> goRight = new ArrayList<>();
        List<CurvePoint> goGoal = new ArrayList<>();
        List<CurvePoint> toPark = new ArrayList<>();

        goMid.add(new CurvePoint(boWei.getPosition()));
        goMid.add(new CurvePoint(-24,59, Math.PI));
        goMid.add(new CurvePoint(-12,36,Math.PI));
        //goLeft.add(new CurvePoint(-36,-16, Math.PI));
        goGoal.add(new CurvePoint(12,30, Math.PI));

        /*
        goMid.add(new CurvePoint(boWei.getPosition()));
        goMid.add(new CurvePoint(-1.031,-6,-2.842));

        goRight.add(new CurvePoint(boWei.getPosition()));
        goRight.add(new CurvePoint(-1.807,-5.686,-2.938));

         */

        //toPark.add(new CurvePoint(-12, -24, Math.PI));
        toPark.add(new CurvePoint(4,-12, Math.PI));
        toPark.add(new CurvePoint(8, -24, Math.PI));
        waitForStart();


        sleep(3000);
        //boWei.grab();
        //boWei.goStiff();
        cases=boWei.pipeline.getRings();
        telemetry.addData("rings0", cases);

        //boWei.grabber.setPosition(0.16);
        //boWei.autonUnflip();
        //boWei.flip();
        //boWei.flippyFlip.setPosition(.88);
        telemetry.update();
        if (cases == 1){
            goGoal.add(new CurvePoint(46, 50, Math.PI)); // 40, 15
        }
        else if (cases == 0){
            goGoal.add(new CurvePoint(0, 46, 0)); //12,-40 could change angle to Math.PI/2 facing sideways
        }
        else if (cases == 4){
            goGoal.add(new CurvePoint(48,48, 0)); //15,-48
        }

        //40, 15
        boWei.followCurveSync(goMid, 8, 0.5,5);
//        while (Math.abs(2.76 - Math.abs(boWei.getHeading())) > 0.05 && opModeIsActive()) {
//            boWei.rightBack.setPower(0.3);
//            boWei.rightFront.setPower(0.3);
//            boWei.leftBack.setPower(-0.3);
//            boWei.leftFront.setPower(-0.3);
//        }
//        boWei.rightBack.setPower(0);
//        boWei.rightFront.setPower(0);
//        boWei.leftBack.setPower(0);
//        boWei.leftFront.setPower(0);

        boWei.turnTo(-2.7, 0.3);
        //telemetry.addData("position",boWei.getPosition());
        boWei.launch.setVelocity(8.4, AngleUnit.RADIANS);
        sleep(2000);
        boWei.collect();
        boWei.upward();
        timer.reset();
        while (ringsShot < 3 && timer.seconds() < 7){
            boolean hasRing = false;
            if (boWei.name.alpha() > 200){
                hasRing = true;
            }
            if (!hasRing && counter){
                ringsShot++;
            }
            counter = hasRing;
        }
        boWei.upwards.setPower(0);
        boWei.spin.setPower(0);
        boWei.launch.setPower(0);
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("Green").setStrokeWidth(1).setStroke("goldenrod").fillCircle(boWei.getX(), boWei.getY(), 5);
        //packet.put("velocity", boWei.launch.getVelocity(AngleUnit.RADIANS));
        //packet.put("rate", rate);
        //packet.put("want rate", 8.6);
        dashboard.sendTelemetryPacket(packet);

        boWei.followCurveSync(goGoal,8, 0.5, 10);
       // boWei.autonUnflip();
        sleep(500);
        //boWei.release();
        sleep(500);
        //boWei.flip();
        toPark.add(0, boWei.getPosition());
        boWei.followCurveSync(toPark,8,0.5,5);
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        //robot.setEndPosition(boWei.getPosition());
    }

}
