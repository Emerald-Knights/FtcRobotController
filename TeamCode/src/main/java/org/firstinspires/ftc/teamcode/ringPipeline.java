package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;

public class ringPipeline extends OpenCvPipeline{
    @Override
    public Mat processFrame(Mat input) {
        //Point xy = new Point(42, 3);
        //Point xx = new Point(114, 30);
        Mat thing = new Mat();
        //Rect rectangle = new Rect(xy, xx);
        //Rect rectangle314 = new Rect(new Point(14, 14), new Point(90, 190));
        //Line x = new Line();
        //Line y = new Line();
        List<MatOfPoint> matList = new ArrayList<>();
        //Imgproc.rectangle(input, rectangle314, new Scalar(235, 83, 162), 5);
        Imgproc.cvtColor(input, thing, Imgproc.COLOR_RGB2YCrCb);

        //Imgproc.findContours(Mat input, List, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE)

        //Core.inRange(thing, colors, colors, thing);

        Core.extractChannel(thing, thing, 1);
        Imgproc.threshold(thing, thing, 160, 220, Imgproc.THRESH_BINARY);
        input.get(160, 120);

        Imgproc.findContours(thing, matList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, matList, -1, new Scalar(255, 0 , 0), 3);

        for(MatOfPoint point: matList){
            Rect drawThing = Imgproc.boundingRect(point);
            Imgproc.rectangle(input, drawThing, new Scalar(0, 0, 0), 3);
        }
        //for(objectInList name: list){
        // do stuff
        //}
        //Imgproc.boundingRect(MatOfPoint) returns Rect
        //Point name = new Point(x, y);
        //Rect rectangle = new Rect(Point1, Point2)
        //Imgproc.rectangle(Mat output, Rect )

        return input;
    }
}
