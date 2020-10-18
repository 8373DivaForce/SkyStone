package org.firstinspires.ftc.teamcode.Autonomous.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Old.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.*;
import org.firstinspires.ftc.teamcode.Functions.Old.SkystoneOpenCVPipe;
import org.firstinspires.ftc.teamcode.Hardware_Maps.BeastBoyHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@Autonomous
@Disabled
public class BeastBoyStone extends LinearOpMode {
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
        telemetry.addData("Status", "Initializing!");
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        //setup initial variables that are defined based on alliance
        FunctionLibrary.Point startPosition = null;
        FunctionLibrary.Point finalPosition = null;
        //setup an array of stones for the 3 closest to the skybridge
        FunctionLibrary.Point[] firstStones = new FunctionLibrary.Point[3];
        //setup an array of stone for the 3 closest to the wall
        FunctionLibrary.Point[] secondStones = new FunctionLibrary.Point[3];
        //setup variable for the camera we are using
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "Front Webcam");
        //setup rectangle size for opencv skystone identification
        float rectHeight = .6f/8f;
        float rectWidth = 1.5f/8f;

        //set the offset between stones
        float offsetX = 1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

        //define a variable for which hook we are using
        //define a variable for our start rotation for later use
        double aproachX = 38;
        double aproachYOffset = 4;
        double startingRotation = 0;
        Point finalFoundationPos = new Point(0,0);
        Point secondFoundationPos = new Point(0,0);
        FunctionLibrary.Point foundationPos = new FunctionLibrary.Point(0,0);
        double foundationTurnAngle = 0;
        double gripperSpeed = 1;
        if (Alliance == 0) {
            //set the offset for the right camera
            offsetX = 0f/8f;
            aproachYOffset = 5;
            //set starting position for blue side
            startPosition = new FunctionLibrary.Point(62.5,-40);

            //tell the robot where the foundation is
            foundationPos = new FunctionLibrary.Point(29,40);
            secondFoundationPos = new Point(31,40);
            finalFoundationPos = new Point(60, foundationPos.y);
            foundationTurnAngle = 110;
            //set the position of all 6 stones
            secondStones[0] = new FunctionLibrary.Point(23,-24);
            firstStones[0] = new FunctionLibrary.Point(23,-42);

            firstStones[2] = new FunctionLibrary.Point(23,-26);
            secondStones[2] = new FunctionLibrary.Point(23,-56);

            firstStones[1] = new FunctionLibrary.Point(23,-34);
            secondStones[1] = new FunctionLibrary.Point(23, -64);

            //set the two parking positions
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(40, 0);
            }
            startingRotation = -90;
            //set which camera we are using
        }
        else if (Alliance == 1) {
            //set the offset for the right camera
            offsetX = 0f/8f;
            aproachX *= -1;
            //set starting position for blue side
            startPosition = new FunctionLibrary.Point(-62.5,-40);

            //tell the robot where the foundation is
            foundationPos = new FunctionLibrary.Point(-29,40);
            secondFoundationPos = new Point(-31,40);
            finalFoundationPos = new Point(-60, foundationPos.y);
            foundationTurnAngle = -110;
            //set the position of all 6 stones
            secondStones[0] = new FunctionLibrary.Point(-24,-24);
            firstStones[0] = new FunctionLibrary.Point(-24,-42);

            firstStones[1] = new FunctionLibrary.Point(-24,-26);
            secondStones[1] = new FunctionLibrary.Point(-24,-56);

            firstStones[2] = new FunctionLibrary.Point(-24,-34);
            secondStones[2] = new FunctionLibrary.Point(-24, -64);

            //set the two parking positions
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-40, 0);
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

        BeastBoyHardware robot = new BeastBoyHardware(this,startingRotation,startPosition,true);
        AutoFunctions auto = new AutoFunctions(robot);
        telemetry.addData("Status", "Done!");
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        waitForStart();
        //robot.dcIntakeRight.setPower(1);
        //robot.dcIntakeLeft.setPower(1);
        int nSwitch = 0;
        int result = 0;
        Point destination;
        Point skystone = new Point(0,0);
        int stone = 0;
        int startState = 0;
        while (opModeIsActive()) {
            startState = nSwitch;
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
                        if (stone == 0) foundationPos = secondFoundationPos;
                        nSwitch++;
                        //close the pipeline to save resources
                        webcam.closeCameraDevice();
                    }
                    break;
                case 1:
                    //line up with the corner of the skystone
                    destination = new Point(aproachX, skystone.y+aproachYOffset);
                    result = auto.gotoPosition(destination,0.5,0.5,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    //turn to face away from the build zone
                    result = auto.rotPID(180,0.5,1,2);
                    if (result < 0) {
                        nSwitch++;
                        robot.dcIntakeLeft.setPower(1);
                        robot.dcIntakeRight.setPower(1);
                    }
                    break;
                case 3:
                    //drive into the stones next to it with the intake in line with the skystone
                    destination = new Point(skystone.x,skystone.y+aproachYOffset);
                    result = auto.gotoPosition(destination,0.5,0.5,180);
                    if (result < 0) nSwitch++;
                    break;
                case 4:
                    //drive into the skystone
                    result = auto.gotoPosition(skystone,0.5,0.5,180);
                    if (!Double.isNaN(robot.blockRange.getDistance(DistanceUnit.INCH))) {
                        robot.dcIntakeRight.setPower(0);
                        robot.dcIntakeLeft.setPower(0);
                    }
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    //turn back to the original rotation
                    result = auto.rotPID(startingRotation,0.5,1,2);
                    //if the distance sensor see the block in the intake, stop the intake
                    if (!Double.isNaN(robot.blockRange.getDistance(DistanceUnit.INCH))) {
                        robot.dcIntakeRight.setPower(0);
                        robot.dcIntakeLeft.setPower(0);
                    }
                    if (result < 0) {
                        nSwitch++;
                        robot.dcIntakeLeft.setPower(0);
                        robot.dcIntakeRight.setPower(0);
                    }
                    break;
                case 6:
                    //line up with the parking position and skystone
                    destination = new Point(finalPosition.x,skystone.y);
                    result = auto.gotoPosition(destination, 0.5, 0.5, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    //turn towards the build zone
                    result = auto.rotPID(-startingRotation,0.25,5,2);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    //realline with the parking position and skystone
                    destination = new Point(finalPosition.x, skystone.y);
                    result = auto.gotoPosition(destination, 0.5, 0.50, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    //align with the parking position and foundation
                    destination = new Point(finalPosition.x, foundationPos.y);
                    result = auto.gotoPosition(destination, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    //line up with the foundation
                    result = auto.gotoPosition(foundationPos, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    //grab the foundation with the servo hooks
                    robot.foundLeft.setPosition(1);
                    robot.foundRight.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 12:
                    //spit the skystone out on to the foundation
                    robot.dcIntakeRight.setPower(gripperSpeed);
                    robot.dcIntakeLeft.setPower(gripperSpeed);
                    if (getRuntime() > 0.5) {
                        nSwitch++;
                        robot.dcIntakeLeft.setPower(0);
                        robot.dcIntakeRight.setPower(0);
                    }
                    break;
                case 13:
                    //close the grabber on it to help align it
                    robot.gBottomLeft.setPosition(0);
                    robot.gBottomRight.setPosition(0);
                    robot.gTopLeft.setPosition(0);
                    robot.gTopRight.setPosition(0);
                    if (getRuntime() > 0.25) nSwitch++;
                    break;
                case 14:
                    //reopen the gripper
                    robot.gBottomLeft.setPosition(1);
                    robot.gBottomRight.setPosition(1);
                    robot.gTopLeft.setPosition(1);
                    robot.gTopRight.setPosition(1);
                    nSwitch++;
                    break;
                case 15:
                    //pull the foundation into the build site
                    destination = new Point(finalFoundationPos.x,foundationPos.y);
                    result = auto.gotoPosition(destination, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 16:
                    result = auto.rotPID(foundationTurnAngle, 0.5, 5, 2);
                    if (result < 0) nSwitch++;
                    break;
                case 17:
                    //release the foundation
                    robot.foundRight.setPosition(0);
                    robot.foundLeft.setPosition(0);
                    if (getRuntime() > 0.25) nSwitch++;
                    break;
                case 18:
                    //move just past the foundation
                    destination = new Point(finalFoundationPos.x,14);
                    result = auto.gotoPosition(destination, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 19:
                    //move in line with parking position
                    destination = new Point(finalPosition.x, 14);
                    result = auto.gotoPosition(destination, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 20:
                    //park
                    result = auto.gotoPosition(finalPosition, 0.5, 0.5, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;


            }
            if (nSwitch != startState) resetStartTime();
        }
    }
}
