package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
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
@Autonomous(name= "opencvSkystoneDetector", group="Sky autonomous")
public class Test extends LinearOpMode {
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
        String[] thingsToControl = {
                "Alliance",
                "Auto",
                "Position",
                "EndPosition"
        };
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        telemetry.addData("Startup", "Vuforia is starting up! Do not stop the robot!");
        if (file.exists()) {
            String[] fileContents = ReadWriteFile.readFile(file).split(",");
            Alliance = Integer.parseInt(fileContents[0]);
            telemetry.addData("Alliance", Alliance);
            Auto = Integer.parseInt(fileContents[1]);
            telemetry.addData("Auto", Auto);
            Position = Integer.parseInt(fileContents[2]);
            telemetry.addData("Position", Position);
            EndPosition = Integer.parseInt(fileContents[3]);
            telemetry.addData("EndPosition", EndPosition);
        } else {
            stop();
        }
        telemetry.update();
        FunctionLibrary.Point startPosition = null;
        FunctionLibrary.Point secondPosition = null;
        FunctionLibrary.Point SkystoneTarget = null;
        FunctionLibrary.Point finalPosition = null;
        FunctionLibrary.Point firstStone = null;
        FunctionLibrary.Point thirdStone = null;
        FunctionLibrary.Point secondStone = null;
        FunctionLibrary.Point[] firstStones = new FunctionLibrary.Point[3];
        FunctionLibrary.Point[] secondStones = new FunctionLibrary.Point[3];
        double grabRotation = 0;
        WebcamName cameraName = null;
        float rectHeight = .6f/8f;
        float rectWidth = 1.5f/8f;

        float offsetX = 1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        Servo hook;
        double startingRotation = 0;
        if (Alliance == 0) {
            offsetX = -1f/8f;
            grabRotation = 15;
            startPosition = new FunctionLibrary.Point(62,-31.5);

            firstStones[2] = new FunctionLibrary.Point(30,-26);
            firstStone = new FunctionLibrary.Point(30,-24);
            secondStones[2] = new FunctionLibrary.Point(30,-52);

            firstStones[1] = new FunctionLibrary.Point(30,-34);
            secondStone = new FunctionLibrary.Point(30,-32);
            secondStones[1] = new FunctionLibrary.Point(30,-60);

            firstStones[0] = new FunctionLibrary.Point(30,-42);
            thirdStone = new FunctionLibrary.Point(30,-40);
            secondStones[0] = new FunctionLibrary.Point(29, -68);

            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(36, 0);
            }
            cameraName = hardwareMap.get(WebcamName.class, "Left Webcam");
        }
        else if (Alliance == 1) {
            offsetX = 1f/8f;
            grabRotation = -15;
            startPosition = new FunctionLibrary.Point(-63,-31.5);

            firstStones[0] = new FunctionLibrary.Point(-30,-26);
            firstStone = new FunctionLibrary.Point(-30,-24);
            secondStones[0] = new FunctionLibrary.Point(-30,-52);

            firstStones[1] = new FunctionLibrary.Point(-30,-34);
            secondStone = new FunctionLibrary.Point(-30,-32);
            secondStones[1] = new FunctionLibrary.Point(-30,-60);

            firstStones[2] = new FunctionLibrary.Point(-30,-42);
            thirdStone = new FunctionLibrary.Point(-30,-40);
            secondStones[2] = new FunctionLibrary.Point(-29, -68);

            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-40, 0);
            }
            cameraName = hardwareMap.get(WebcamName.class, "Right Webcam");

        }
        float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
        float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        SkystoneOpenCVPipe pipeline = new SkystoneOpenCVPipe(leftPos, midPos, rightPos, rectWidth, rectHeight);
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,startPosition,0);
        AutoFunctions auto = new AutoFunctions(robot);
        hook = robot.sRStoneHook;
        if (Alliance == 0) hook = robot.sLStoneHook;
        while (!isStarted() && !isStopRequested()) {
            int[] values = pipeline.getValues();
            telemetry.addData("Values: ", values[0] + " " + values[1] + " " + values[2]);
            telemetry.update();
        }
        runtime.reset();
        int nSwitch = 0;
        int result = 0;
        FunctionLibrary.Point destination;
        FunctionLibrary.Point skystone = new FunctionLibrary.Point(0,0);
        int stone= -1;
        while (opModeIsActive()) {
            switch(nSwitch) {
                case 0:
                    int[] values = pipeline.getValues();
                    if (values[0] == 0) stone = 0;
                    else if (values[1] == 0) stone = 1;
                    else if (values[2] == 0) stone = 2;
                    if (stone != -1) {
                        skystone = firstStones[stone];
                        nSwitch++;
                        webcam.closeCameraDevice();
                    }
                    break;
                case 1:
                    result = auto.gotoPosition(skystone,1,1,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    hook.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case -3:
                    result = auto.rotPID(grabRotation,0.5,5,5);
                    if (result < 0) nSwitch++;
                    break;
                case 4:
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination ,1 ,1 ,0);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    destination = new FunctionLibrary.Point(finalPosition.x, -10);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    destination = new FunctionLibrary.Point(finalPosition.x, 20);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    hook.setPosition(0);
                    if (getRuntime() > 1) {
                        nSwitch++;
                        skystone = secondStones[stone];
                    }
                    break;
                case 8:
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 1) nSwitch++;
                    break;
                case 9:
                    result = auto.gotoPosition(skystone, 1, 1 ,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    hook.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 11:
                    result = auto.rotPID(grabRotation,0.5,5,5);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    destination = new FunctionLibrary.Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 13:
                    destination = new FunctionLibrary.Point(finalPosition.x, 20);
                    result = auto.gotoPosition(destination,1 ,1 ,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 14:
                    hook.setPosition(0);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 15:
                    result = auto.gotoPosition(finalPosition, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;


            }
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
        auto.close();
    }


    //detection pipeline
}