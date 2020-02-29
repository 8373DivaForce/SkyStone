package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Functions.SkystoneOpenCVPipe;
import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@TeleOp
public class KissBotStone extends LinearOpMode {
    private final int rows = 640;
    private final int cols = 480;
    @Override
    public void runOpMode() throws InterruptedException {
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
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Front Webcam");
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
        Point foundationPos = new Point(0,0);
        if (Alliance == 0) {
            //set the offset according to the left webcam
            offsetX = 1f/8f;
            grabRotation = 15;
            //set the start position
            startPosition = new Point(65,-40);

            //set the position of all of the stones
            firstStones[2] = new Point(24,-24);
            secondStones[2] = new Point(24,-46);

            firstStones[1] = new Point(24,-33);
            secondStones[1] = new Point(24,-55);

            firstStones[0] = new Point(24,-41);
            secondStones[0] = new Point(24, -65);

            foundationPos = new Point(32,48);
            //define the two parking positions
            if (EndPosition == 0) {
                finalPosition = new Point(58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new Point(41, 0);
            }
            //define which camera we are using
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            //set the offset for the right camera
            offsetX = -0.5f/8f;
            //set the angle we rotate to in order to grab a block
            grabRotation = -15;
            //set starting position for blue side
            startPosition = new Point(-65,-40);

            foundationPos = new Point(-32,48);
            //set the position of all 6 stones
            firstStones[0] = new Point(-24,-28);
            secondStones[0] = new Point(-24,-52);

            firstStones[1] = new Point(-24,-36);
            secondStones[1] = new Point(-24,-60);

            firstStones[2] = new Point(-24,-44);
            secondStones[2] = new Point(-24, -68);

            //set the two parking positions
            if (EndPosition == 0) {
                finalPosition = new Point(-58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new Point(-40, 0);
            }
            startingRotation = 90;
            //set which camera we are using
        }
        //set the positions of all the boxes for opencv skystone identificaiton
        float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
        float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
        float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
        //get the screen on the phone for the camera to output to
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //intiailize opencv camera factory using the designated webcam
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        //initialize our detector pipeline
        SkystoneOpenCVPipe pipeline = new SkystoneOpenCVPipe(leftPos, midPos, rightPos, rectWidth, rectHeight);
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        NewKissHardware robot = new NewKissHardware(this,startingRotation,startPosition, true);
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        int nSwitch = 0;
        int result = 0;
        Point destination = new Point(0,0);
        int stone = -1;
        double rampingDistance = 1;
        Point skystone = startPosition;
        Point[] points = new Point[0];
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
                    result = auto.gotoPosition(skystone,0.5,0.5,startingRotation,rampingDistance);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    points = new Point[4];
                    points[0] = skystone;
                    points[1] = new Point(finalPosition.x, skystone.y);
                    points[2] = new Point(finalPosition.x, foundationPos.y);
                    points[3] = foundationPos;
                    result = auto.purePursuit(points,6,0.5,0.5,startingRotation,telemetry);
                    if (result < 0) {
                        nSwitch++;
                        skystone = secondStones[stone];
                    }
                    break;
                case 3:
                    points = new Point[4];
                    points[0] = foundationPos;
                    points[1] = new Point(finalPosition.x, foundationPos.y);
                    points[2] = new Point(finalPosition.x, skystone.y);
                    points[3] = skystone;
                    result = auto.purePursuit(points,6,0.5,0.5,startingRotation,telemetry);
                    if (result < 0) nSwitch++;
                    break;
                case 4:
                    points = new Point[4];
                    points[0] = skystone;
                    points[1] = new Point(finalPosition.x, skystone.y);
                    points[2] = new Point(finalPosition.x, foundationPos.y);
                    points[3] = foundationPos;
                    result = auto.purePursuit(points,6,0.5,0.5,startingRotation,telemetry);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    points = new Point[3];
                    points[0] = foundationPos;
                    points[1] = new Point(foundationPos.y,finalPosition.x);
                    points[2] = finalPosition;
                    result = auto.purePursuit(points,6,0.5,0.5,startingRotation,telemetry);
                    if (result < 0) nSwitch++;
                    break;

            }
            telemetry.addData("Position", robot.getX() + ", " + robot.getY());
            telemetry.addData("Rotation", robot.getWorldRotation());
            telemetry.update();
        }
    }
}
