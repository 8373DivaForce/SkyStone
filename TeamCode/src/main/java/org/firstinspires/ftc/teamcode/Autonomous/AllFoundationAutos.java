package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

import java.io.File;

@Autonomous
public class AllFoundationAutos extends LinearOpMode {
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
        FunctionLibrary.Point secondPosition = null;
        FunctionLibrary.Point thirdPosition = null;
        FunctionLibrary.Point fourthPosition = null;
        double startingRotation = 0;
        if (Alliance == 0) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(62, 15);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(60, 36);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(30, 0);
            }
            startingRotation = -90;
            secondPosition = new FunctionLibrary.Point(30,48);
            thirdPosition = new FunctionLibrary.Point(24,48);
            fourthPosition = new FunctionLibrary.Point(36,48);

        }
        else if (Alliance == 1) {
            if (Position == 0) {
                startPosition = new FunctionLibrary.Point(-62, 15);
            } else if (Position == 1) {
                startPosition = new FunctionLibrary.Point(-62, 38);
            }
            if (EndPosition == 0) {
                finalPosition = new FunctionLibrary.Point(-62, 0);
            } else if (EndPosition == 1) {
                finalPosition = new FunctionLibrary.Point(-30, 0);
            }
            startingRotation = 90;
            secondPosition = new FunctionLibrary.Point(-30,48);
            thirdPosition = new FunctionLibrary.Point(-24,48);
            fourthPosition = new FunctionLibrary.Point(-36,48);
        }
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,startPosition,startingRotation);
        AutoFunctions auto = new AutoFunctions(robot);
        FunctionLibrary.motorMovement liftControl = new FunctionLibrary.motorMovement(100,robot.dcLift);
        liftControl.limits(robot.upperLimitSwitch, robot.lowerLimitSwitch);
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        waitForStart();
        int nSwitch = 1;
        int result = 0;
        int result2 = 0;
        double x;
        double y;
        FunctionLibrary.Point destination = new FunctionLibrary.Point(0,0);
        while (opModeIsActive()) {

            switch (nSwitch) {
                case 1:
                    if (result > -1)
                        result = auto.gotoPosition(secondPosition,1,1,startingRotation);
                    if (result2 > -1)
                        result2 = liftControl.move_encoder_to_position(6500, 1, 5, 100, false);
                    if (result < 0 && result2 < 0) nSwitch++;
                    break;
                case 2:
                    result = auto.gotoPosition(thirdPosition,0.3,0.5,startingRotation);
                    if (result < 0 ) {
                        nSwitch++;
                        resetStartTime();
                    }
                    break;
                case 3:
                    robot.sLFoundHook.setPosition(0);
                    robot.sRFoundHook.setPosition(0);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 4:
                    destination = new FunctionLibrary.Point(thirdPosition.x-(startingRotation/Math.abs(startingRotation))*6,thirdPosition.y);
                    result = auto.gotoPosition(destination, 0.5, 1, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 5:
                    result = auto.rotPID(180+(startingRotation/Math.abs(startingRotation))*40, 0.5, 20, 2);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    result = auto.rotPID(-startingRotation, 0.5, 10, 3);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    robot.sLFoundHook.setPosition(0.9);
                    robot.sRFoundHook.setPosition(0.9);
                    if (getRuntime() > 1) nSwitch++;
                    break;
                case 8:
                    result = auto.rotPID(-startingRotation,0.5, 3, 2);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    destination = new FunctionLibrary.Point((-startingRotation/Math.abs(startingRotation))*18 + thirdPosition.x,thirdPosition.y);
                    result = auto.gotoPosition(destination, 1, 1, -startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    destination = new FunctionLibrary.Point((-startingRotation/Math.abs(startingRotation))*12 + thirdPosition.x,thirdPosition.y);
                    result = auto.gotoPosition(destination,1,1,-startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    result = liftControl.move_encoder_to_position(-2500, 1, 3, 50, false);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    result = auto.gotoPosition(finalPosition, 1, 1, 180);
                    if (result < 0) nSwitch++;
                    break;

            }
            telemetry.addData("switch", nSwitch);
            telemetry.addData("result", nSwitch);
            telemetry.update();
        }
    }
}
