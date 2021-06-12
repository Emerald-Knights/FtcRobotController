package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import static org.firstinspires.ftc.teamcode.utilities.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point;

@TeleOp(name="drive2electricBugaloo", group="f")
@Disabled
public class Drive2electricBugaloo extends LinearOpMode{
	robot2Wheel boWei = new robot2Wheel();

	@Override
	public void runOpMode(){
		//float currentVex = 0;
		//boWei.location.setPosition(robot.endX, robot.endY, robot.endAngle);
		DcMotor launch;
		DcMotorEx spin;
		DcMotor upwards = null;
		launch = (DcMotorEx) hardwareMap.get(DcMotor.class, "launch");
		spin = hardwareMap.get(DcMotorEx.class,"spin");
		upwards=hardwareMap.get(DcMotor.class, "upwards");
		spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		spin.setPower(0);
		spin.setDirection(DcMotorSimple.Direction.FORWARD);
		spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		upwards.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		upwards.setPower(0);
		upwards.setDirection(DcMotorSimple.Direction.REVERSE);
		launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		launch.setPower(0);
		launch.setDirection(DcMotorSimple.Direction.FORWARD);
		launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


		waitForStart();
//		boolean isMoving = false;
//		boolean aIsPressed = false;
//		boolean leftPressed = false;
//		boolean rightPressed=false;
//		boolean slowmodeActive = false;
//		boolean dLeftHeld=false;
//		boolean dRightHeld=false;
//		boolean dUpHeld = false;
//		boolean ddownHeld = false;
//		double toggle = 1;
//		boolean lbumpPressed=false;
//		boolean yIsPressed=false;
//
//		boolean distanceBasedShoot=false;
//		double rate=1;
//
//		double oldAngle = 0;
//		boolean bIsPressed=false;
//		boolean collecting=false;
//
//		boolean fieldOriented= false;
//
//		boolean prevRStick=false;
//		boolean prevLStick=false;
//
//		boolean aimLock=false;

		//FtcDashboard dashboard = FtcDashboard.getInstance();

		while(opModeIsActive()){
			//upwards.setPower(0.5);
			if (gamepad1.a){
				spin.setPower(0.5);
				telemetry.speak("AAAAAAA");
			}
			if (gamepad1.b){
				upwards.setPower(0.5);
			}
			if (upwards.getPower() <0) {
				telemetry.speak("yes");
			}
			if (gamepad1.x){
				launch.setPower(0.5);
			}


//			double lx=gamepad1.left_stick_x;
//			double ly=-gamepad1.left_stick_y;
//			double rx=gamepad1.right_stick_x;
//
//			boolean lsb=gamepad1.left_stick_button;
//			boolean rsb=gamepad1.right_stick_button;
//
//			boolean lbump= gamepad1.left_bumper;
//
//			//deadzone
//			if (Math.abs(lx) <= 0.15) {
//				lx = 0;
//			}
//
//			if (Math.abs(ly) <= 0.15) {
//				ly = 0;
//			}
//
//			if (Math.abs(rx) <= 0.15) {
//				rx = 0;
//			}
///*
//            //shooter levels
//            if(gamepad2.dpad_up){
//                boWei.leftLift.setPosition(.46); //third thing
//                boWei.rightLift.setPosition(.56);
//            }
//            if(gamepad2.dpad_down){
//                boWei.leftLift.setPosition(.43); //powershot
//                boWei.rightLift.setPosition(.53);
//            }
//
//
//			if (gamepad2.dpad_up && !dUpHeld){
//				rate++;
//				if (rate > 6){
//					rate=6;
//				}
//				dUpHeld = true;
//			}
//			if (!gamepad2.dpad_up){
//				dUpHeld = false;
//			}
//			if (gamepad2.dpad_down && !ddownHeld){
//				rate--;
//				if (rate < 0){
//					rate= 0;
//				}
//
//				ddownHeld = true;
//			}
//			if (!gamepad2.dpad_down){
//				ddownHeld = false;
//			}
//
//			if(gamepad2.dpad_left && !dLeftHeld){
//				rate-=.1;
//				if(rate<0){
//					rate=0;
//				}
//				dLeftHeld=true;
//			}
//			if(!gamepad2.dpad_left){
//				dLeftHeld=false;
//			}
//			if(gamepad2.dpad_right&& !dRightHeld){
//				rate+=.1;
//				if(rate>6){
//					rate=6;
//				}
//
//				dRightHeld=true;
//			}
//			if(!gamepad2.dpad_right){
//				dRightHeld=false;
//			}
//
//
//			//collection
//			if (gamepad2.left_bumper){
//				boWei.reverseCollect();
//			}
//			else if (gamepad2.right_bumper){
//				boWei.collect();
//			}
//			else {
//				boWei.spin.setPower(0);
//			}
//
//			//launcher
//			if(gamepad2.a && !aIsPressed && !gamepad2.start){
//				if (isMoving) {
//					telemetry.speak("Anthony is gay");
//					isMoving = false;
//				}
//				else if (!isMoving){
//					telemetry.speak("pew pew");
//					isMoving = true;
//				}
//				aIsPressed = true;
//			}
//			if (!gamepad2.a){
//				aIsPressed = false;
//			}
//			if(gamepad2.y && !yIsPressed){
//				distanceBasedShoot=!distanceBasedShoot;
//				yIsPressed=true;
//			}
//			if(!gamepad2.y){
//				yIsPressed=false;
//			}
//
//			if(isMoving){
//				if(distanceBasedShoot){
//					//rate=.0136904762 * (Math.hypot(goal.x-boWei.getX(), goal.y-boWei.getY()))+3.542857143;
//					rate=boWei.getRate();
//
//				}
//				boWei.launch.setVelocity(rate, AngleUnit.RADIANS);
//			}
//			else{
//				boWei.launch.setVelocity(0, AngleUnit.RADIANS);
//			}
//
//			//upwards
//			if(gamepad2.b&&!bIsPressed){
//				if(collecting){
//					boWei.upwards.setPower(0);
//					collecting=false;
//				}
//				else if(!collecting){
//					boWei.upward();
//					collecting=true;
//				}
//				bIsPressed=true;
//			}
//			if(!gamepad2.b){
//				bIsPressed=false;
//			}
//
//			if (lsb && !leftPressed){
//				if (!slowmodeActive){
//					toggle = 0.5;
//					slowmodeActive = true;
//					telemetry.speak("Slowmode, activated");
//				}
//				else if (slowmodeActive) {
//					toggle = 1;
//					slowmodeActive = false;
//					telemetry.speak("Normal mode, activated");
//				}
//				leftPressed = true;
//			}
//
//			if(rsb && !rightPressed){
//				aimLock= !aimLock;
//
//				rightPressed=true;
//			}
//
//			if(!lsb){
//				leftPressed = false;
//			}
//			if(!rsb){
//				rightPressed=false;
//			}
//
//			if(lbump && !lbumpPressed){
//				fieldOriented=!fieldOriented;
//				if(fieldOriented){
//					telemetry.speak("field oriented, activated");
//				}
//				else{
//					telemetry.speak("robot oriented, activated");
//				}
//				lbumpPressed=true;
//			}
//			if(!lbump){
//				lbumpPressed=false;
//			}
//
//			if(aimLock){
//				double angleDiff= boWei.getAngleDiff();
//				if(Math.abs(angleDiff)>.05){
//					int d=0;
//					rx= angleDiff/Math.abs(angleDiff)*( Math.abs(angleDiff)/6.0 + .3);
//
//				}
//				else{
//					rx=0;
//				}
//			}
//
//			double lf;
//			double lb;
//			double rf;
//			double rb;
//
//			double ratio;
//
//			if(fieldOriented){
//				double magnitude= Math.hypot(ly, lx);
//				double angle= angleWrap(Math.atan2(ly, -lx)-Math.PI/4 + boWei.getHeading());
//
//				lf = (magnitude*Math.sin(angle) +rx)*.8;
//				lb = (magnitude*Math.cos(angle) +rx)*.8;
//				rf = (magnitude*Math.cos(angle) -rx)*.8;
//				rb = (magnitude*Math.sin(angle) -rx)*.8;
//
//				double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
//				double mag=Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
//				ratio = mag / max;
//				if (max == 0) {
//					ratio=0;
//				}
//			}
//
//			else{
//				lf = ly + rx + lx;
//				lb = ly + rx - lx;
//				rf = ly - rx - lx;
//				rb = ly - rx + lx;
//
//
//				double max = Math.max(Math.max(Math.abs(lb), Math.abs(lf)), Math.max(Math.abs(rb), Math.abs(rf)));
//				double magnitude = Math.sqrt((lx * lx) + (ly * ly) + (rx * rx));
//				ratio = .8*magnitude / max;
//				if (max == 0) {
//					ratio=0;
//				}
//			}
//
//			double angle = oldAngle - boWei.getHeading();
//			oldAngle = boWei.getHeading();
//
//
//			//telemetry.addData("LF", lf);
//			//telemetry.addData("LB", lb);
//			//telemetry.addData("RF", rf);
//			//telemetry.addData("RB", rb);
//
////            telemetry.addData("left", -boWei.leftOdo.getCurrentPosition());
////            telemetry.addData("right", boWei.rightOdo.getCurrentPosition());
////            telemetry.addData("horizontal", boWei.horizontalOdo.getCurrentPosition());
//			telemetry.addData("Diff:", (boWei.location.positionLeft + boWei.location.forwardEncoderToRadian *  angle) + (boWei.location.positionRight - boWei.location.forwardEncoderToRadian * -angle));
//
//			telemetry.addData("Position", ("("+round1000(boWei.getX())+", "+round1000( boWei.getY() ) + ", " + round1000(boWei.getHeading())+")"));
//
//            /*           telemetry.addData("rad/s", boWei.launch.getVelocity(AngleUnit.RADIANS));
//            telemetry.addData("tick/s", boWei.launch.getVelocity());
//            telemetry.addData("rate", rate);
//            telemetry.addData("Field oriented: ", fieldOriented);
//
//
//             */
//			telemetry.update();
//
//
//			TelemetryPacket packet = new TelemetryPacket();
//			packet.fieldOverlay().setFill("Green").setStrokeWidth(1).setStroke("goldenrod").fillCircle(boWei.getX(), boWei.getY(), 5);
//			//dashboard.sendTelemetryPacket(packet);
//
//
//
////            boWei.leftFront.setPower(lf * ratio * toggle);
////            boWei.leftBack.setPower(lb * ratio * toggle);
////            boWei.rightFront.setPower(rf * ratio * toggle);
////            boWei.rightBack.setPower(rb * ratio * toggle);
//
//			prevLStick=lsb;
//			prevRStick=rsb;
		}

	}
}
