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
            //stop the program if there isn't a file
            stop();
        }
        FunctionLibrary.Point startPosition = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point foundationAproachingPos = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point foundationPosition = new FunctionLibrary.Point(0,0);
        FunctionLibrary.Point finalPosition = new FunctionLibrary.Point(0,0);
        double startingRotation = 0;
        if (Alliance == 0) {
            foundationPosition = new FunctionLibrary.Point(36, 48);
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
            foundationPosition = new FunctionLibrary.Point(-36, 48);
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
        waitForStart();
        double result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                case 0:
                    result = auto.gotoPosition(foundationAproachingPos, 1, 1, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = inout.move_using_encoder(300,1, 2, 50, false);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    result = lift.move_using_encoder(5300, 1, 3, 50, false);
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    result = auto.gotoPosition(foundationPosition, 1, 1, startingRotation);
                    if (result < 0) nSwitch++;
                    break;
            }
        }

    }
}
