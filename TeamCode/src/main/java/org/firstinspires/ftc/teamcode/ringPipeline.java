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
        //input is the image that the phone camera sees, with RGB values for each pixel

        //just making points for learning purposes
        Point xy = new Point(42, 3);
        Point xx = new Point(114, 30);

        //Mat are objects, they are basically 2d arrays but they represent images
        Mat thing = new Mat(); //a new temporary mat (image)

        //making rectangles for practice. Rectangles have 2 points for top left and bottom right as parameters.
        Rect rectangle = new Rect(xy, xx);
        Rect rectangle314 = new Rect(new Point(14, 14), new Point(90, 190));
        //Line x = new Line();
        //Line y = new Line();

        //new arrayList of MatOfPoint. MatOfPoint is basically a contour object
        List<MatOfPoint> matList = new ArrayList<>();

        //practice drawing rectangles
        //Imgproc.rectangle(input, rectangle314, new Scalar(235, 83, 162), 5);

        //This line makes it so the image now has YCrCb color values instead of RGB values (google YCrCb). This is saved in a new image, thing
        Imgproc.cvtColor(input, thing, Imgproc.COLOR_RGB2YCrCb);

        //We only are going to be looking at the first (technically 2nd because the start at 0 thing) of YCrCb (so the Cr part).
        //if we were to extract the 1 channel of rgb, that would be g (green). 0 is R, 2 is B
        Core.extractChannel(thing, thing, 1);

        //checks what range of values pixels are in. If they are outside of 160 to 220 for the Cr channel (see previous line), they are removed.
        //displaying "thing" now would show white pixels for all pixels that are red/ orange, while everything else would be black
        Imgproc.threshold(thing, thing, 160, 220, Imgproc.THRESH_BINARY);

        //Find the edges of the image, aka where "thing" has white to black or vise versa transitions, and puts it into a list
        Imgproc.findContours(thing, matList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw the contours not on "thing", but on "input", the original image of what the camera sees. The final image has what the camera sees with lines drawn on it
        Imgproc.drawContours(input, matList, -1, new Scalar(100, 100 , 100), 3, 8);

        //for all the contours (saved as MatOfPoint objects) draw rectangles around it
        for(MatOfPoint point: matList){
            Rect drawThing = Imgproc.boundingRect(point);
            Imgproc.rectangle(input, drawThing, new Scalar(100, 100, 100), 3, 8);
        }

        //return the modified input image for the phone to display on screen
        return input;


    }
}
