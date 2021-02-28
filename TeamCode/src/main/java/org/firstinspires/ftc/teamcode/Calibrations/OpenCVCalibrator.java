
package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GameChangerOpenCVPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@TeleOp
public class OpenCVCalibrator extends OpMode {
    OpenCvCamera webcam;
    GameChangerOpenCVPipeline pipeline;
    ElapsedTime timer = new ElapsedTime();
    File colorLimits = AppUtil.getInstance().getSettingsFile("OpenCVColorLimits");
    public void toCSV(Scalar mins, Scalar maxes) {
        toCSV(mins.val,maxes.val);
    }
    public void toCSV(double[] mins, double[] maxes) {
        String output = "";
        output += mins[0] + ",";
        output += mins[1] + ",";
        output += mins[2] + ",";
        output += maxes[0] + ",";
        output += maxes[1] + ",";
        output += maxes[2];
        ReadWriteFile.writeFile(colorLimits,output);
    }
    public Scalar[] fromCSV() {
        if (!colorLimits.exists()) {
            Scalar[] scalars = new Scalar[]{new Scalar(0,0,0),new Scalar(255,255,255)};
            toCSV(scalars[0],scalars[1]);
            colorLimits = AppUtil.getInstance().getSettingsFile("openCVColorLimits");
            return scalars;
        }
        String input = ReadWriteFile.readFile(colorLimits);
        String[] vals = input.split(",");
        Scalar[] scalars = new Scalar[2];
        double[] pVals = new double[6];
        for (int i = 0; i < 6; i++) {
            pVals[i] = Double.parseDouble(vals[i]);
        }
        scalars[0] = new Scalar(pVals[0],pVals[1],pVals[2]);
        scalars[1] = new Scalar(pVals[3],pVals[4],pVals[5]);
        return scalars;
    }
    double[] mins = {0,0,0};
    double[] maxes = {255,255,255};
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //intiailize opencv camera factory using the designated webcam
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        //initialize our detector pipeline
        Scalar[] limits = fromCSV();
        mins = limits[0].val;
        maxes = limits[1].val;
        pipeline = new GameChangerOpenCVPipeline(limits[0],limits[1]);
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC

        fromCSV();
    }
    int selectedMin = 0;
    int selectedMax = 0;
    @Override
    public void init_loop() {
        if (getRuntime() >= 0.1 && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.x || gamepad1.b || gamepad1.y || gamepad1.a)) {
            if (gamepad1.dpad_up) {
                mins[selectedMin] = (mins[selectedMin] + 1) % 256;
            } else if (gamepad1.dpad_down) {
                mins[selectedMin] = (mins[selectedMin] - 1);
            if (mins[selectedMin] < 0) mins[selectedMin] = 255;
            } else if (gamepad1.dpad_left) {
                selectedMin = (selectedMin - 1);
            if (selectedMin < 0) selectedMin = 2;
            } else if (gamepad1.dpad_right) {
                selectedMin = (selectedMin + 1) % 3;
            } else if (gamepad1.y) {
                maxes[selectedMax] = (maxes[selectedMax] + 1) % 256;
            } else if (gamepad1.a) {
                maxes[selectedMax] = (maxes[selectedMax] - 1);
            if (maxes[selectedMax] < 0) maxes[selectedMax] = 255;
            } else if (gamepad1.x) {
                selectedMax = (selectedMax - 1);
            if (selectedMax < 0) selectedMax = 2;
            } else if (gamepad1.b) {
                selectedMax = (selectedMax + 1) % 3;
            }
            pipeline.setMins(new Scalar(mins));
            pipeline.setMaxes(new Scalar(maxes));
            resetStartTime();
        }
        if (timer.seconds() >= 0.5) {
            toCSV(mins,maxes);
            timer.reset();
        }
        String minText = "";
        String maxText = "";
        for (int i = 0; i < 3; i++) {
            if (selectedMin == i) {
                minText += ">";
            }
            if (selectedMax == i) {
                maxText += ">";
            }
                minText += mins[i];
                maxText += maxes[i];
            if (i != 2) {
                minText += ", ";
                maxText += ", ";
            }
        }
        telemetry.addData("Mins",minText);
        telemetry.addData("Maxes",maxText);
        telemetry.addData("Ratio: ", pipeline.getRatio());
        telemetry.update();
    }
    @Override
    public void loop() {

    }
    @Override
    public void stop() {
        toCSV(new Scalar(mins), new Scalar(maxes));
        webcam.closeCameraDevice();
    }
}
