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

@Autonomous(name = "justParkBlue", group = "hmmm7")
public class forDeltaV extends LinearOpMode {
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
        List<CurvePoint> toPark = new ArrayList<>();

        //toPark.add(new CurvePoint(-12, -24, Math.PI));
        toPark.add(new CurvePoint(-62, 44, Math.PI));
        toPark.add(new CurvePoint(8, -24, Math.PI));
        waitForStart();


        sleep(25000);
        //boWei.grab();
        //boWei.goStiff();
        //boWei.grabber.setPosition(0.16);
        //boWei.autonUnflip();
        //boWei.flip();
        //boWei.flippyFlip.setPosition(.88);
        telemetry.update();


        //40, 15
        toPark.add(0, boWei.getPosition());
        boWei.followCurveSync(toPark,8,0.5,5);
        boWei.rightBack.setPower(0);
        boWei.rightFront.setPower(0);
        boWei.leftBack.setPower(0);
        boWei.leftFront.setPower(0);
        //robot.setEndPosition(boWei.getPosition());
    }

}
