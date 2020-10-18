package org.firstinspires.ftc.teamcode.Autonomous.Old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Old.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.BeastBoyHardware;

import java.io.File;

@Autonomous
@Disabled
public class AllBridgeAutos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        //read the file we store the information on which autonomous we are running
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        if (file.exists()) {
            String[] fileContents = ReadWriteFile.readFile(file).split(",");
            Alliance = Integer.parseInt(fileContents[0]);
            Auto = Integer.parseInt(fileContents[1]);
            Position = Integer.parseInt(fileContents[2]);
            EndPosition = Integer.parseInt(fileContents[3]);
        } else {
            //if the file isn't there, stop the program
            stop();
        }
        AutoValues autoValues = new AutoValues(telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        //use the information read from the file to choose the start position, final position, and starting rotation
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
        //initialize robot
        BeastBoyHardware robot = new BeastBoyHardware(this,startingRotation,startPosition,true);
        //initialize autonomous functions
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        int nSwitch = 1;
        int result = 0;
        double x;
        double y;
        FunctionLibrary.Point destination = new FunctionLibrary.Point(0,0);
        while (opModeIsActive()) {

            switch (nSwitch) {
                //if we are lining up against the neutral bridge, drive forwards and stop
                case 1:
                    destination = new FunctionLibrary.Point(finalPosition.x,startPosition.y);
                    result = auto.gotoPosition(destination,0.5,1,startingRotation,12);
                    if (result < 0) nSwitch++;
                    break;
                //drive over and under the bridge
                case 2:
                    result = auto.gotoPosition(finalPosition, 0.5, 1, startingRotation,12);
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
