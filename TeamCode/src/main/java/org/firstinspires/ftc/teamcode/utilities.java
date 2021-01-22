package org.firstinspires.ftc.teamcode;

public class utilities {
    public static double checkAngle(double angle){
        return angle % Math.PI;
    }
    public static double angleWrap(double angle){
        while(angle>Math.PI){
            angle-=2*Math.PI;
        }
        while(angle<-Math.PI){
            angle+=2*Math.PI;
        }
        return angle;
    }
}
