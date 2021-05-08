package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.MatOfPoint;

import java.util.ArrayList;
import java.util.List;
import static org.firstinspires.ftc.teamcode.utilities.*;

public class ringPipeline extends OpenCvPipeline{
    //List<food> shoppingList = newArrayList<>();
    //shoppingList.get(0);
    int xc = 200;
    int yc = 160;
    Point crosshairPoint = new Point(xc, yc);
    int crosshairLength = 10;
    Rect crosshair = new Rect(new Point(xc-crosshairLength/2, yc-crosshairLength/2), new Point(xc + crosshairLength/2, yc + crosshairLength/2));
    int rings = 0;
    double rRatio;

    double[] ybrV={0, 0, 0};
    @Override
    public Mat processFrame(Mat input) {
        Core.flip(input, input, -1);
        //input is the image that the phone camera sees, with RGB values for each pixel

        //just making points for learning purposes
        //Point xy = new Point(42, 3);
        //Point xx = new Point(114, 30);

        //Mat are objects, they are basically 2d arrays but they represent images
        Mat thing = new Mat(); //a new temporary mat (image)

        //making rectangles for practice. Rectangles have 2 points for top left and bottom right as parameters.
        //Rect rectangle = new Rect(xy, xx);
        //Rect rectangle314 = new Rect(new Point(14, 14), new Point(90, 190));
        //Line x = new Line();
        //Line y = new Line();

        //new arrayList of MatOfPoint. MatOfPoint is basically a contour object
        List<MatOfPoint> matList = new ArrayList<>();

        //practice drawing rectangles
        //Imgproc.rectangle(input, rectangle314, new Scalar(235, 83, 162), 5);

        //This line makes it so the image now has YCrCb color values instead of RGB values (google YCrCb). This is saved in a new image, thing
        Imgproc.cvtColor(input, thing, Imgproc.COLOR_RGB2YCrCb);

        ybrV=thing.get(yc, xc);

        //We only are going to be looking at the first (technically 2nd because the start at 0 thing) of YCrCb (so the Cr part).
        //if we were to extract the 1 channel of rgb, that would be g (green). 0 is R, 2 is B
        //Core.extractChannel(thing, thing, 1);

        //checks what range of values pixels are in. If they are outside of 160 to 220 for the Cr channel (see previous line), they are removed.
        //displaying "thing" now would show white pixels for all pixels that are red/ orange, while everything else would be black
        //Core.inRange(thing, new Scalar(0, 160, 40), new Scalar(255, 180, 100), thing); //75
        Core.inRange(thing, new Scalar(0, 140, 50), new Scalar(255, 200, 130), thing); //cb:100
        //Core.bitwise_not(thing, thing);

        //Imgproc.threshold(thing, thing, 160, 180, Imgproc.THRESH_BINARY); //220

        Imgproc.dilate(thing, thing, Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(12,15)));


        //Find the edges of the image, aka where "thing" has white to black or vise versa transitions, and puts it into a list
        Imgproc.findContours(thing, matList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw the contours not on "thing", but on "input", the original image of what the camera sees. The final image has what the camera sees with lines drawn on it
        Imgproc.drawContours(input, matList, -1, new Scalar(54, 192, 243), 3, 8);

        //for all the contours (saved as MatOfPoint objects) draw rectangles around it
        Rect max = new Rect(new Point(0,0), new Point(1,1));

        Imgproc.rectangle(input, crosshair, new Scalar(4, 233, 78), 3, 8);
        for(MatOfPoint point: matList){
            //xc, yc for crosshair x and y coordinate
            Rect drawThing = Imgproc.boundingRect(point);
            //if (drawThing.area() > max.area() && drawThing.contains(crosshairPoint)){

            if (drawThing.area() > max.area() && touches(drawThing, crosshair)){
                max = drawThing;
            }
            //rectangleName.contains(point);
            //drawThing.contains() -> boolean
        }
        //if area> small number -> if ratio closer to 1/8 or 1/2 -> , else (area <small) -> no ring
        // 0__________ 2/8 (close to 1/8) 0_______5/8 (close 1/2)
        rRatio=0;
        int smallest = 2;
        if (max.area() > smallest) {

            double ratio = (1.0 * max.height)/(max.width) * 1.0;
            rRatio=ratio;
            //.8 ish
            if (ratio > .5 && ratio < .9) {
                rings = 4;
            }
            else if (ratio > .3 && ratio <= .5) {
                rings = 1;
            }
            else{
                rings = 0;
            }
        }
        else{
            rings = 0;
        }

        Imgproc.rectangle(input, max, new Scalar(255, 175, 66), 3, 8);


        //return the modified input image for the phone to display on screen
        return input;

    }
    public int getRings () {
        return rings;
    }
    public double getRatio(){
        return rRatio;
    }
    public double[] getColor(){
        return ybrV;
    }
}