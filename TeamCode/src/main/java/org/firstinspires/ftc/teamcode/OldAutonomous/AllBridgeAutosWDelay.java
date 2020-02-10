package org.firstinspires.ftc.teamcode.OldAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

import java.io.File;

@Autonomous
public class AllBridgeAutosWDelay extends LinearOpMode {
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
        FunctionLibrary.Point finalPosition = null;
        double startingRotation = 0;
        if (Alliance == 0) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(62, -14);
            } else if (Position == 2) {
                startPosition = new FunctionLibrary.Point(62, 15);
            } else if (Position == 3) {
                startPosition = new FunctionLibrary.Point(62, 38);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(36, 0);
            }
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, -38);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, -14);
            } else if (Position == 2) {
                startPosition = new FunctionLibrary.Point(-62, 15);
            } else if (Position == 3) {
                startPosition = new FunctionLibrary.Point(-62, 38);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-36, 0);
            }
            startingRotation = 90;
        }
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,startPosition,startingRotation);
        AutoFunctions auto = new AutoFunctions(robot);
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        waitForStart();
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
                    destination = new FunctionLibrary.Point(-6,startPosition.y);
                    result = auto.gotoPosition(destination,1,1,startingRotation);
                    x = robot.getX()-destination.x;
                    y = robot.getY()-destination.y;
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    resetStartTime();
                    nSwitch++;
                    break;
                case 3:
                    if (getRuntime() > 10) nSwitch++;
                    break;
                case 4:
                    destination = new FunctionLibrary.Point(finalPosition.x,startPosition.y);
                    result = auto.gotoPosition(destination,1,1,startingRotation);
                    x = robot.getX()-destination.x;
                    y = robot.getY()-destination.y;
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    result = auto.gotoPosition(finalPosition, 1, 1, startingRotation);
                    x = robot.getX()-finalPosition.x;
                    y = robot.getY()-finalPosition.y;
                    if (result < 0) nSwitch++;
                    break;
            }
            telemetry.addData("switch", nSwitch);
            telemetry.addData("result", nSwitch);
            telemetry.update();
        }
    }
}
