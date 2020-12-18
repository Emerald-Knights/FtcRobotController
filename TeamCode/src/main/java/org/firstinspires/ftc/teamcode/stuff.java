package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "auton", group = "auton")
public class stuff extends LinearOpMode{
    robot boWei = new robot();
    @Override
    public void runOpMode(){
        boWei.init(hardwareMap, this);
        waitForStart();
        boWei.parking(3000, 1);
    }


}
