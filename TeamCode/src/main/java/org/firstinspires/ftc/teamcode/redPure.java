package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class redPure extends LinearOpMode {
    robot boWei = new robot();
    @Override
    public void runOpMode() throws InterruptedException {
        boWei.init(hardwareMap, this);
        boWei.initOpenCV();

        boWei.path.add(new CurvePoint(boWei.getX(), boWei.getY(), boWei.getHeading()));
        boWei.path.add(new CurvePoint(0, -24, 0));

        waitForStart();

        boWei.followCurveSync(boWei.path, 8, .8, 4);
    }
}
