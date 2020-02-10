package org.firstinspires.ftc.teamcode.OldAutonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

import java.io.File;

@Autonomous
@Disabled
public class AllStoneAutos extends LinearOpMode {
    private final double mmPerInch = 25.4;
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
        double startingRotation = 0;
        if (Alliance == 0) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(62, -12.5);
                secondPosition = new FunctionLibrary.Point(40, -28);
                SkystoneTarget = new FunctionLibrary.Point(24,-46);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(37.5, 0);
            }
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, -38);
                secondPosition = new FunctionLibrary.Point(-40, -54);
                SkystoneTarget = new FunctionLibrary.Point(-24,-68);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, -14);
                secondPosition = new FunctionLibrary.Point(-40, -36);
                SkystoneTarget = new FunctionLibrary.Point(-24,-46);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-58, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-36, 0);
            }
            startingRotation = 90;
        }
        D1V4hardware robot = new D1V4hardware(this,startPosition,startingRotation);
        AutoFunctions auto = new AutoFunctions(robot);
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcUpDown1, robot.dcUpDown2);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(10,robot.dcOpenClose);
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
                    result = inoutControl.move_using_encoder(300, 0.5, 5, 20, false);
                    if (result < 0) {
                        nSwitch++;
                    }
                    break;
                case 1:
                    destination = new FunctionLibrary.Point(startPosition.x,secondPosition.y);
                    result = auto.gotoPosition(destination,1,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    result = auto.gotoPosition(secondPosition, 0.75, 1, startingRotation);
                    resetStartTime();
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    robot.updateVuforia();
                    //if the skystone is found, head towards it
                    //otherwise, assume it's the third and head towards that
                    if(robot.VuMarkPositions.containsKey("Stone Target")) {
                        resetStartTime();
                        VectorF stonePos = robot.VuMarkPositions.get("Stone Target");

                        if (Alliance == 0) {
                            y = (stonePos.get(1)/mmPerInch)+robot.getY()-2;
                            x = (stonePos.get(0)/mmPerInch)+robot.getX()-4;
                        } else {
                            y = -(stonePos.get(1)/mmPerInch)+robot.getY()+2;
                            x = -(stonePos.get(0)/mmPerInch)+robot.getX()+4;
                        }

                        SkystoneTarget = new FunctionLibrary.Point(x,y);
                        Log.d("BothAutoVuforia", "x: " + SkystoneTarget.x + "y: " + SkystoneTarget.y);
                        nSwitch++;
                    } else if (getRuntime() > 2) {
                        //skystone position was set above, so the program will just use that
                        resetStartTime();
                        nSwitch++;
                        if (Math.abs(SkystoneTarget.y+70) < 10) {
                            nSwitch = -10;
                        }
                    }
                    break;
                case -10:
                    destination = new FunctionLibrary.Point(secondPosition.x, SkystoneTarget.y+24);
                    result = auto.gotoPosition(destination,0.75,1,180);
                    if (result < 0) nSwitch--;
                    break;
                case -11:
                    destination = new FunctionLibrary.Point(SkystoneTarget.x, SkystoneTarget.y+8);
                    result = auto.gotoPosition(SkystoneTarget, 0.75, 1, 180);
                    if (result < 0 ) nSwitch = 6;
                    break;
                case 4:
                    destination = new FunctionLibrary.Point(secondPosition.x, SkystoneTarget.y);
                    result = auto.gotoPosition(destination,0.75,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    destination = new FunctionLibrary.Point(SkystoneTarget.x,SkystoneTarget.y);
                    result = auto.gotoPosition(destination,0.75,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    //close the grabber on it
                    result = openCloseControl.move_using_encoder(12500, 1, 8,10,true);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    destination = new FunctionLibrary.Point(finalPosition.x,SkystoneTarget.y);
                    result = auto.gotoPosition(destination,1,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    destination = new FunctionLibrary.Point(finalPosition.x, 12);
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    result = openCloseControl.move_using_encoder(-2000,1,5,20,false);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    result = auto.gotoPosition(finalPosition,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
            }
            telemetry.addData("switch", nSwitch);
            telemetry.addData("result", nSwitch);
            telemetry.update();
        }
    }
}
