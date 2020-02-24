package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.SkystoneOpenCVPipe;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.Kisshardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(group="Sky autonomous")
public class WorkingTwoStoneAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;


    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        //setup variables that we read from the file
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        if (file.exists()) {
            String[] fileContents = ReadWriteFile.readFile(file).split(",");
            Alliance = Integer.parseInt(fileContents[0]);
            Auto = Integer.parseInt(fileContents[1]);
            Position = Integer.parseInt(fileContents[2]);
            EndPosition = Integer.parseInt(fileContents[3]);
        } else {
            stop();
        }
        AutoValues autoValues = new AutoValues(telemetry);
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        //setup initial variables that are defined based on alliance
        FunctionLibrary.Point startPosition = null;
        FunctionLibrary.Point finalPosition = null;
        //setup an array of stones for the 3 closest to the skybridge
        FunctionLibrary.Point[] firstStones = new FunctionLibrary.Point[3];
        //setup an array of stone for the 3 closest to the wall
        FunctionLibrary.Point[] secondStones = new FunctionLibrary.Point[3];
        //setup variable for what way to rotate to grab a stone
        double grabRotation = 0;
        //setup variable for the camera we are using
        WebcamName cameraName = null;
        //setup rectangle size for opencv skystone identification
        float rectHeight = .6f/8f;
        float rectWidth = 1.5f/8f;

        //set the offset between stones
        float offsetX = 1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        //define a variable for which hook we are using
        Servo hook;
        //define a variable for our start rotation for later use
        double startingRotation = 0;
        if (Alliance == 0) {
            //set the offset according to the left webcam
            offsetX = -1f/8f;
            grabRotation = 15;
            //set the start position
            startPosition = new FunctionLibrary.Point(63,-30);

            //set the position of all of the stones
            firstStones[2] = new FunctionLibrary.Point(35,-24);
            secondStones[2] = new FunctionLibrary.Point(32,-46);

            firstStones[1] = new FunctionLibrary.Point(35,-33);
            secondStones[1] = new FunctionLibrary.Point(32,-55);

            firstStones[0] = new FunctionLibrary.Point(35,-41);
            secondStones[0] = new FunctionLibrary.Point(32, -65);

            //define the two parking positions
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(41, 0);
            }
            //define which camera we are using
            cameraName = hardwareMap.get(WebcamName.class, "Left Webcam");
        }
        else if (Alliance == 1) {
            //set the offset for the right camera
            offsetX = 1f/8f;
            //set the angle we rotate to in order to grab a block
            grabRotation = -15;
            //set starting position for blue side
            startPosition = new FunctionLibrary.Point(-63,-33);

            //set the position of all 6 stones
            firstStones[0] = new FunctionLibrary.Point(-35,-23);
            secondStones[0] = new FunctionLibrary.Point(-32,-44);

            firstStones[1] = new FunctionLibrary.Point(-35,-31);
            secondStones[1] = new FunctionLibrary.Point(-32,-52);

            firstStones[2] = new FunctionLibrary.Point(-34,-41);
            secondStones[2] = new FunctionLibrary.Point(-32, -60);

            //set the two parking positions
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-40, 0);
            }
            //set which camera we are using
            cameraName = hardwareMap.get(WebcamName.class, "Right Webcam");

        }
        //set the positions of all the boxes for opencv skystone identificaiton
        float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
        float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
        //get the screen on the phone for the camera to output to
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //intiailize opencv camera factory using the designated webcam
        webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        //initialize our detector pipeline
        SkystoneOpenCVPipe pipeline = new SkystoneOpenCVPipe(leftPos, midPos, rightPos, rectWidth, rectHeight);
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        //initialize hardware map with start position and rotation
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,startPosition,0);
        //intialize autofunctions class
        AutoFunctions auto = new AutoFunctions(robot);
        //set the hook to the hook it needs to use for the alliance it's on
        hook = robot.sRStoneHook;
        if (Alliance == 0) hook = robot.sLStoneHook;
        while (!isStarted() && !isStopRequested()) {
            int[] values = pipeline.getValues();
            telemetry.addData("Values: ", values[0] + " " + values[1] + " " + values[2]);
            telemetry.update();
        }
        //reset the runtime for our functions
        runtime.reset();
        int nSwitch = 0;
        int result = 0;
        FunctionLibrary.Point destination;
        FunctionLibrary.Point skystone = new FunctionLibrary.Point(0,0);
        int stone= -1;
        while (opModeIsActive()) {
            switch(nSwitch) {
                case 0:
                    //read from our pipeline and set the corresponding skystone value
                    int[] values = pipeline.getValues();
                    if (values[0] == 0) stone = 0;
                    else if (values[1] == 0) stone = 1;
                    else if (values[2] == 0) stone = 2;
                    if (stone != -1) {
                        //set our skystone variable to the positoin of the skystone
                        skystone = firstStones[stone];
                        nSwitch++;
                        //close the pipeline to save resources
                        webcam.closeCameraDevice();
                    }
                    break;
                case 1:
                    //goto the position of the skystone
                    result = auto.gotoPosition(skystone,1,1,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    //drop the skystone hook
                    hook.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 3:
                    //rotate to help ensure that we grab the block
                    result = auto.rotPID(grabRotation,1,5,1);
                    if (result < 0) nSwitch++;
                    break;
                case 4:
                    //drag the block over in line with the part of the skybridge we park under
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination ,1 ,1 ,0);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    //drag the block to the other side through the part of the bridge we park under
                    destination = new FunctionLibrary.Point(finalPosition.x, 10);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    //release the block
                    hook.setPosition(0);
                    if (getRuntime() > 1) {
                        nSwitch++;
                        //set the skystone to the second stone we are going for
                        skystone = secondStones[stone];
                    }
                    break;
                case 7:
                    //move in line with the skystone
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 1) nSwitch++;
                    break;
                case 8:
                    //line up with the skystone along both axis
                    result = auto.gotoPosition(skystone, 1, 1 ,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    //grab the skystone
                    hook.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 10:
                    //turn to ensure that we grab the skystone
                    result = auto.rotPID(grabRotation,1,5,1);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    //drag the block over and in line with the part under the bridge that we park
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    //go past the bridge in order to drop off the stone
                    destination = new FunctionLibrary.Point(finalPosition.x, 10.1);
                    result = auto.gotoPosition(destination,1 ,1 ,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 13:
                    //release the skystone
                    hook.setPosition(0);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 14:
                    //park under the bridge
                    result = auto.gotoPosition(finalPosition, 0.5, 1, 0);
                    if (result < 0) nSwitch++;
                    break;


            }

        }
        //close the auto datalogger
        auto.close();
    }


    //detection pipeline
}