package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp
public class OpenCvTest extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "left"), cameraMonitorViewId);
        camera.openCameraDevice();
        camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        waitForStart();
        SamplePipeline pipe = new SamplePipeline();
        pipe.rectangles.add(new rectangle(new Point(910,290), new Point(1140,410),new Scalar(0,255,0),4));
        pipe.rectangles.add(new rectangle(new Point(470,280), new Point(690,410),new Scalar(0,255,0),4));
        camera.setPipeline(pipe);
        int x1 = 20;
        int y1 = 20;
        int x2 = 1260;
        int y2 = 700;
        int rectangle = 0;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean xPressed = false;
        boolean bPressed = false;
        boolean rightBumperPressed = false;
        while (opModeIsActive()) {
            if (!rightBumperPressed && gamepad1.right_bumper) {
                rectangle += 1;
                rightBumperPressed = true;
            } else if (rightBumperPressed && !gamepad1.right_bumper) rightBumperPressed = false;
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                x1 -= 10;
                dpadLeftPressed = true;
            } else if(!gamepad1.dpad_left && dpadLeftPressed) dpadLeftPressed = false;
            if (gamepad1.dpad_right && !dpadRightPressed) {
                x1 += 10;
                dpadRightPressed = true;
            } else if(!gamepad1.dpad_right && dpadRightPressed) dpadRightPressed = false;
            if (gamepad1.dpad_up && !dpadUpPressed) {
                y1 += 10;
                dpadUpPressed = true;
            } else if(!gamepad1.dpad_up && dpadUpPressed) dpadUpPressed = false;
            if (gamepad1.dpad_down && !dpadDownPressed) {
                y1 -= 10;
                dpadDownPressed = true;
            } else if(!gamepad1.dpad_down && dpadDownPressed) dpadDownPressed = false;

            if (gamepad1.x && !xPressed) {
                x2 -= 10;
                xPressed = true;
            } else if(!gamepad1.x && xPressed) xPressed = false;
            if (gamepad1.b && !bPressed) {
                x2 += 10;
                bPressed = true;
            } else if(!gamepad1.b && bPressed) bPressed = false;
            if (gamepad1.y && !yPressed) {
                y2 += 10;
                yPressed = true;
            } else if(!gamepad1.y && yPressed) yPressed = false;
            if (gamepad1.a && !aPressed) {
                y2 -= 10;
                aPressed = true;
            } else if(!gamepad1.a && aPressed) aPressed = false;
            telemetry.addData("Point 1", x1 + ", " + y1);
            telemetry.addData("Point 2", x2 + ", " + y2);
            int valIter = 0;
            int aIter = 0;
            for (double[] val : pipe.values) {
                aIter = 0;
                if (val != null)
                for (double d : val) {
                    telemetry.addData("Block " + valIter +" Value " + aIter, d);
                    aIter++;
                }
                valIter++;
            }


            telemetry.update();
        }
    }

}
class rectangle {
    public final Point point1;
    public final Point point2;
    public final Scalar scalar;
    public final int thickness;
    public rectangle(Point point1, Point point2, Scalar scalar, int thickness) {
        this.point1 = point1;
        this.point2 = point2;
        this.scalar = scalar;
        this.thickness = thickness;
    }
}
class SamplePipeline extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    public ArrayList<double[]> values = new ArrayList<>();
    public ArrayList<rectangle> rectangles = new ArrayList<>();
    @Override
    public Mat processFrame(Mat input)
    {

        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */
        if (rectangles.size() > 0)
            for (int j = 0; j < rectangles.size(); j++) {
            rectangle rectangle = rectangles.get(j);
            Imgproc.rectangle(input,rectangle.point1,rectangle.point2,rectangle.scalar, 4);
            for (double x = rectangle.point1.x; x < rectangle.point2.x; x++) {
                for (double y = rectangle.point1.y; y < rectangle.point2.y; y++) {

                    if (values.size()-1 < j) {
                        values.add(j,input.get((int)x,(int)y));
                    } else {
                        double[] tempValues = input.get((int)x,(int)y);
                        double[] tempValues2 = values.get(j);
                        double[] tempValues3 = new double[4];
                        if (tempValues != null && tempValues2 != null) {
                            for (int i = 0; i < 3; i++) {
                                tempValues3[i] = (tempValues[i] + tempValues2[i])/2;
                            }
                            values.set(j, tempValues3);
                        }
                    }

                }
            }
        }
        /*
        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);
        */
        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }
}
