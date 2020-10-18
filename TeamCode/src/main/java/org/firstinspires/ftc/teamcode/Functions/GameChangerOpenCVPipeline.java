package org.firstinspires.ftc.teamcode.Functions;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


public class GameChangerOpenCVPipeline extends OpenCvPipeline
{

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

    public double getRatio() {
        return ratio;
    }
    public GameChangerOpenCVPipeline(Scalar mins, Scalar maxes) {
        this.mins = mins;
        this.maxes = maxes;
    }
    public void setMins(Scalar mins) {
        this.mins = mins;
    }
    public void setMaxes(Scalar maxes) {
        this.maxes = maxes;
    }
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

        //color diff cb.
        //lower cb = more blue = skystone = white
        //higher cb = less blue = yellow stone = grey
        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb

        //b&w
        Core.inRange(yCbCrChan2Mat, mins, maxes,yCbCrChan2Mat);
        //outline/contour

        Imgproc.findContours(yCbCrChan2Mat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours
        //Log.d("openCV", "maxes: " + maxes.val[0] + ", " + maxes.val[1] + ", " + maxes.val[2]);
        //Log.d("openCV", "mins: " + mins.val[0] + ", " + mins.val[1] + ", " + mins.val[2]);
        boolean runOnce = false;
        Log.d("openCV",contoursList.size() + "");
        for (MatOfPoint contour : contoursList) {
            rect = Imgproc.boundingRect(contour);
            if (!runOnce || (rect.width > largestRect.width && rect.height > largestRect.height)) {
                runOnce = true;
                largestRect = rect;
            }
        }
        res.release();
        res = new Mat();
        Core.bitwise_and(input,input,res,yCbCrChan2Mat);


        if (runOnce) {
            ratio = largestRect.width/largestRect.height;
            Imgproc.rectangle(res, largestRect, new Scalar(0,0,255));
        } else {
            ratio = 0;
        }
        return res;
    }

}

