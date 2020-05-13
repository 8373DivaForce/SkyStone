package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.AutoValues;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

import java.io.File;

@Autonomous
@Disabled
public class AllFoundationAutos extends LinearOpMode {
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

        FunctionLibrary.Point startPosition = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point foundationAproachingPos = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point foundationPosition = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point finalPosition = new FunctionLibrary.Point(0,0);
        double startingRotation = 0;
        if (Alliance == 0) {
            foundationPosition = new FunctionLibrary.Point(28, 46);
            foundationAproachingPos = new FunctionLibrary.Point(48, 48);
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(62, 15);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(62, 42);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(36, 0);
            }
            startingRotation = -90;
        }
        else if (Alliance == 1) {
            foundationPosition = new FunctionLibrary.Point(-28, 46);
            foundationAproachingPos = new FunctionLibrary.Point(-48, 48);
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, 15);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, 42);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-36, 0);
            }
            startingRotation = 90;
        }
        int nSwitch = 0;
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this, startPosition,startingRotation);
        AutoFunctions auto = new AutoFunctions(robot);

        FunctionLibrary.motorMovement lift = new FunctionLibrary.motorMovement(100, robot.dcLift, robot.dcLift2);
        lift.limits(robot.upperLimitSwitch, robot.lowerLimitSwitch);
        FunctionLibrary.motorMovement inout = new FunctionLibrary.motorMovement(100, robot.dcInOut);
        FunctionLibrary.motorMovement gripper = new FunctionLibrary.motorMovement(100, robot.dcOpenClose);
        waitForStart();
        double result = 0;
        double gripperState = 0;
        FunctionLibrary.Point destination = new FunctionLibrary.Point(0,0);
        while (opModeIsActive()) {
            if (gripperState > -1) {
                gripperState = gripper.move_using_encoder(-5500,1,4,10,false);
            }
            telemetry.addData("gripper state",gripperState);
            telemetry.update();
            switch (nSwitch) {
                case 0:
                    result = auto.gotoPosition(foundationAproachingPos, 1, 1, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = inout.move_using_encoder(1400,1, 2, 50, false);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    result = lift.move_using_encoder(3000, 1, 3, 20, false);
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    if (gripperState < 1) nSwitch++;
                    break;
                case 4:
                    result = auto.gotoPosition(foundationPosition, 0.5, 0.5, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    result = lift.move_using_encoder(-2800,1,3,20, false);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    destination = new FunctionLibrary.Point(startPosition.x, foundationPosition.y);
                    result = auto.gotoPosition(destination, 0.5, 1);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    result = inout.move_using_encoder(-1000,1,4,30,false);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    result = lift.move_using_encoder(1800, 1, 3, 20, false);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    destination = new FunctionLibrary.Point(startPosition.x, 20);
                    result = auto.gotoPosition(destination, 1, 1, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    destination = new FunctionLibrary.Point(finalPosition.x, 20);
                    result = auto.gotoPosition(destination,0.5,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    result = lift.move_using_encoder(-1100,1,3,20,false);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    result = auto.gotoPosition(finalPosition,1,1,startingRotation);
                    if (result < 0) nSwitch++;
                    break;
            }
        }

    }
}
