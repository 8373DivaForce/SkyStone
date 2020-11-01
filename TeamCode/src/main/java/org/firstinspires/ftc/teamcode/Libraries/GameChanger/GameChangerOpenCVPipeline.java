package org.firstinspires.ftc.teamcode.Libraries.GameChanger;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class GameChangerOpenCVPipeline extends OpenCvPipeline
{

    //defining objects to avoid memory leaks
    Mat yCbCrChan2Mat = new Mat();
    Mat all = new Mat();
    Mat res = new Mat();
    Rect largestRect = new Rect();
    double ratio = 0;
    Rect rect;
    List<MatOfPoint> contoursList = new ArrayList<>();
    boolean hasRan = false;
    private Scalar mins;
    private Scalar maxes;

    //output given to program
    public double getRatio() {
        return ratio;
    }
    //initialize with given color ranges
    public GameChangerOpenCVPipeline(Scalar mins, Scalar maxes) {
        this.mins = mins;
        this.maxes = maxes;
    }
    //set the limits after start (used for calibration)
    public void setMins(Scalar mins) {
        this.mins = mins;
    }
    public void setMaxes(Scalar maxes) {
        this.maxes = maxes;
    }
    //processing loop
    @Override
    public Mat processFrame(Mat input)
    {
        hasRan = true;
        contoursList.clear();
        largestRect = null;
        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */

        //Convert RGB image to yCbCrChan (contrast based image)
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb

        //Get a mesh of black and white of the parts that fit within color range
        Core.inRange(yCbCrChan2Mat, mins, maxes,yCbCrChan2Mat);

        //outlines of shapes that fit within color range
        Imgproc.findContours(yCbCrChan2Mat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object

        //Overlay contours on image shown to driver for debugging and configuration
        Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours;
        boolean runOnce = false;

        //iterate through the contours or "shapes"
        for (MatOfPoint contour : contoursList) {
            //Take the outline and make a rectangle that the contours fit inside of
            rect = Imgproc.boundingRect(contour);
            //if the rectangle is larger than the previous ones, run this
            if (!runOnce || (rect.width > largestRect.width && rect.height > largestRect.height)) {
                //tell the program that we have actually gotten a rectangle this run
                runOnce = true;
                largestRect = rect;
            }
        }
        res.release();
        res = new Mat();
        //combine the black and white mesh with the original input image to show the objects it detects in the color range
        Core.bitwise_and(input,input,res,yCbCrChan2Mat);


        //if the program has found a rectangle run this
        if (runOnce) {
            //set the ratio to the width/height to find the stack size
            ratio = largestRect.width/largestRect.height;
            //draw a rectangle around said object in the output frame
            Imgproc.rectangle(res, largestRect, new Scalar(0,0,255));
        } else {
            //set the ratio to zero since it doesn't see any rings
            ratio = 0;
        }
        //return output image for debugging
        return res;
    }

}

