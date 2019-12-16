package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

import java.io.File;

@Autonomous
public class AllStoneAutos extends LinearOpMode {
    private final double mmPerInch = 25.4;
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4hardware robot = new D1V4hardware(this);
        AutoFunctions auto = new AutoFunctions(robot);
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcUpDown1, robot.dcUpDown2);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(3,robot.dcOpenClose);
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
                finalPosition = new FunctionLibrary.Point(37.5, -12);
            }
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, -14);
                secondPosition = new FunctionLibrary.Point(-40, -36);
                SkystoneTarget = new FunctionLibrary.Point(-24,-46);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-36, -12);
            }
            startingRotation = 90;
        }
        robot.initVuforia(hardwareMap);
        telemetry.addData("Startup", "Ready to start!");
        telemetry.update();
        robot.setRotation(startingRotation);
        robot.setPosition(startPosition);
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
                    result = inoutControl.move_using_encoder(200, 0.5, 5, 20, false);
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
                    }
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
                    result = openCloseControl.move_using_encoder(-1000,1,5,20,false);
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
