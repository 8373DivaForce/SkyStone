package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.vuforia.PositionalDeviceTracker;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

import java.io.File;

@Autonomous
public class AllStoneAutosDouble extends LinearOpMode {
    private final double mmPerInch = 25.4;
    @Override
    public void runOpMode() throws InterruptedException {
        String[] thingsToControl = {
                "Alliance",
                "Auto",
                "Position",
                "EndPosition"
        };
        String cameraName = null;
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
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
        FunctionLibrary.Point secondStoneOffset = null;
        double secondParkOffset = 0;
        Servo stoneServo = null;
        double startingRotation = 0;
        int sideMulti = 1;
        if (Alliance == 0) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(62, -12.5);
                secondPosition = new FunctionLibrary.Point(40, -28);
                SkystoneTarget = new FunctionLibrary.Point(24,-40);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(55, 10);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(33, 0);
                secondParkOffset = -1;
            }
            secondStoneOffset = new FunctionLibrary.Point(0,-30);
            sideMulti = -1;
            cameraName = "Left Webcam";
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, -12.5);
                secondPosition = new FunctionLibrary.Point(-40, -28);
                SkystoneTarget = new FunctionLibrary.Point(-24,-40);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-55, 10);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-33, 0);
                secondParkOffset = 3;
            }
            secondStoneOffset = new FunctionLibrary.Point(4,-32);
            cameraName = "Right Webcam";
            startingRotation = 90;
        }
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,startPosition,startingRotation,cameraName);
        if (Alliance == 0) stoneServo = robot.sLStoneHook;
        if (Alliance == 1) stoneServo = robot.sRStoneHook;
        AutoFunctions auto = new AutoFunctions(robot);
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcLift);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(10,robot.dcOpenClose);
        telemetry.addData("Startup", "Vuforia is starting up! Do not stop the robot!");
        robot.initVuforia(hardwareMap);
        telemetry.addData("Startup", "Ready to start!");
        telemetry.update();
        waitForStart();
        robot.SkystoneTrackables.activate();
        int nSwitch = 0;
        int result = 0;
        double x;
        double y;
        FunctionLibrary.Point destination = new FunctionLibrary.Point(0,0);
        while (opModeIsActive()) {

            switch (nSwitch) {
                case 0:
                    destination = new FunctionLibrary.Point(startPosition.x+5*sideMulti,secondPosition.y);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = auto.gotoPosition(secondPosition, 0.75, 1, 0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    robot.updateVuforia();
                    //if the skystone is found, head towards it
                    //otherwise, assume it's the third and head towards that
                    if(robot.VuMarkPositions.containsKey("Stone Target")) {
                        resetStartTime();
                        VectorF stonePos = robot.VuMarkPositions.get("Stone Target");

                        if (Alliance == 0) {
                            y = (stonePos.get(1)/mmPerInch)+3+robot.getY();
                            x = (stonePos.get(0)/mmPerInch)+robot.getX();
                        } else {
                            x = -(stonePos.get(1)/mmPerInch)+robot.getX()+18;
                            y = (stonePos.get(0)/mmPerInch)+robot.getY()+13;
                        }

                        SkystoneTarget = new FunctionLibrary.Point(x,y);
                        Log.d("BothAutoVuforia", "x: " + SkystoneTarget.x + "y: " + SkystoneTarget.y);
                        nSwitch++;
                        telemetry.addData("Position", x + ", " + y);
                        telemetry.update();
                    } else if (getRuntime() > 2) {
                        //skystone position was set above, so the program will just use that
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
                case 3:
                    result = auto.gotoPosition(SkystoneTarget, 0.75,1,0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 4:
                    stoneServo.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 5:
                    result = auto.rotPID(60*sideMulti*-1,0.75,5,3);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    destination = new FunctionLibrary.Point(finalPosition.x, SkystoneTarget.y);
                    result = auto.gotoPosition(destination,1,1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    destination = new FunctionLibrary.Point(finalPosition.x, 36);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    stoneServo.setPosition(0);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 9:
                    destination = new FunctionLibrary.Point(finalPosition.x, SkystoneTarget.y+secondStoneOffset.y);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    destination = new FunctionLibrary.Point(SkystoneTarget.x+secondStoneOffset.x, SkystoneTarget.y+secondStoneOffset.y);
                    result = auto.gotoPosition(destination, 0.75, 1, 0);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    stoneServo.setPosition(1);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 12:
                    result = auto.rotPID(45*sideMulti*-1,0.76,5,3);
                    if (result < 0) nSwitch++;
                    break;
                case 13:
                    destination = new FunctionLibrary.Point(finalPosition.x+secondParkOffset, SkystoneTarget.y+secondStoneOffset.y);
                    result = auto.gotoPosition(destination,1,1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 14:
                    destination = new FunctionLibrary.Point(finalPosition.x+secondParkOffset, 36);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 15:
                    stoneServo.setPosition(0);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 16:
                    destination = new FunctionLibrary.Point(finalPosition.x+secondParkOffset,finalPosition.y);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;


            }
            /*telemetry.addData("switch", nSwitch);
            telemetry.addData("result", nSwitch);
            telemetry.update();
             */
        }
    }
}
