package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;

public class utilities {

    /**
     * ppl online say utility classes are bad so yall can break my bad habbit next year lmao
     *
     *
     */
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
    //find the intersection of a circle and a line using the quadratic formula
    public static List<Point> proximityPathIntersects(Point center, double radius, Point lineStart, Point lineEnd) {
        //ensures that the slope !=0
        if (Math.abs(lineStart.y - lineEnd.y) < .003) {
            lineStart.y = lineEnd.y + .003;
        }
        if (Math.abs(lineStart.x - lineEnd.x) < .003) {
            lineStart.x = lineEnd.x + .003;
        }
        double slopeLine=(lineEnd.y-lineStart.y)/(lineEnd.x-lineStart.x);

        //translates the line start so that it is relative to the center of the circle (circle center is at (0,0) relative to line start)
        double transX1=lineStart.x-center.x;
        double transY1=lineStart.y-center.y;
/*
        double transX2=lineEnd.x-center.x;
        double transY2=lineEnd.y-center.y;

 */

        //using linestart as point in point slope intercept form
        //using quad to find x with x^2+y^2=r^2 and y-y1=m(x-x1)
        double quadA=1.0+Math.pow(slopeLine, 2);
        double quadB= -2.0*Math.pow(slopeLine, 2)*transX1+2.0*slopeLine*transY1;
        double quadC=Math.pow(slopeLine, 2.0)*Math.pow(transX1, 2)-2.0*slopeLine*transY1*transX1+Math.pow(transY1, 2.0)-Math.pow(radius, 2.0);

        double minX=Math.min(lineStart.x, lineEnd.x);
        double maxX=Math.max(lineStart.x, lineEnd.x);

        double minY=Math.min(lineStart.y, lineEnd.y);
        double maxY=Math.max(lineStart.y, lineEnd.y);
        List<Point> intersections = new ArrayList<>();

        try {
            double point1x=(-quadB+Math.sqrt(Math.pow(quadB,2.0)-4.0*quadA*quadC))/2/quadA;
            //point slope form
            double point1y= slopeLine*(point1x-transX1)+transY1;

            //translate it back to normal coordinate plane
            point1x+=center.x;
            point1y+=center.y;

            //point has to be within the line
            if(point1x> minX && point1x<maxX && point1y>minY && point1y<maxY){
                //add point to list
                intersections.add(new Point(point1x, point1y));
            }
        }
        catch (Exception e){

        }
        try{
            double point2x=(-quadB-Math.sqrt(Math.pow(quadB,2.0)-4.0*quadA*quadC))/2/quadA;
            double point2y= slopeLine*(point2x-transX1)+transY1;


            point2x+=center.x;
            point2y+=center.y;

            if(point2x> minX && point2x<maxX && point2y>minY && point2y<maxY){
                intersections.add(new Point(point2x, point2y));
            }

        }
        catch (Exception e){

        }
        return intersections;
    }
    //finds distance from (x1,y1) to (x2,y2)
    public static double distanceToPoint (double x1, double y1, double x2, double y2){
        return Math.hypot(x2-x1, y2-y1);
    }
    public static double round1000(double num){
        return Math.round(num*1000.0)/1000.0;
    }
    public static boolean touches (Rect r1, Rect r2){
        if(r2.br().x < r1.tl().x  || r1.br().x < r2.tl().x){
            return false;
        }
        if(r2.br().y < r1.tl().y  || r1.br().y < r2.tl().y){
            return false;
        }
        return true;
    }
}
