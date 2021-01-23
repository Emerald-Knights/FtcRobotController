package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

public class CurvePoint {
    double x;
    double y;
    double heading;
    public CurvePoint(double x, double y, double heading){
        this.x=x;
        this.y=y;
        this.heading=heading;
    }
    public CurvePoint(CurvePoint thisPoint) {
        this.x = thisPoint.x;
        this.y = thisPoint.y;
        this.heading = thisPoint.heading;
    }
    public Point toPoint(){
        return new Point(x, y);
    }
    public void setPoint(Point point){
        this.x=point.x;
        this.y=point.y;
    }
}
