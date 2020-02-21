package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

import java.io.File;

@Autonomous
public class AllBridgeAutos extends LinearOpMode {
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
        int nSwitch = 1;
        int result = 0;
        double x;
        double y;
        FunctionLibrary.Point destination = new FunctionLibrary.Point(0,0);
        while (opModeIsActive()) {

            switch (nSwitch) {
                case 1:
                    destination = new FunctionLibrary.Point(finalPosition.x,startPosition.y);
                    result = auto.gotoPosition(destination,1,1,startingRotation);
                    x = robot.getX()-destination.x;
                    y = robot.getY()-destination.y;
                    if (result < 0) nSwitch++;
                    break;
                case 2:
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
