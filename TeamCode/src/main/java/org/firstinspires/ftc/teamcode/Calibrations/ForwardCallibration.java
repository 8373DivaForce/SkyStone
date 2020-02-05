package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.Kisshardware;

@Autonomous(group = "Calibration")
public class ForwardCallibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this, 0,0,0,"Left Webcam");
        AutoFunctions auto = new AutoFunctions(robot);
        String choice = "Forward";
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) choice = "Forward";
            else if (gamepad1.dpad_down) choice = "Backwards";
            else if (gamepad1.dpad_left) choice = "Left";
            else if (gamepad1.dpad_right) choice = "Right";
            telemetry.addData("choice", choice);
            telemetry.update();
        }
        FunctionLibrary.Point destination = null;
        if (choice == "Forward") destination = new FunctionLibrary.Point(0, 24);
        else if (choice == "Backwards") destination = new FunctionLibrary.Point(0, -24);
        else if (choice == "Left") destination = new FunctionLibrary.Point(-24,0);
        else if (choice == "Right") destination = new FunctionLibrary.Point(24,0);

        int nSwitch = 0;
        double result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                case 0:
                    result = auto.gotoPosition(destination, 0.5, 0.5, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = auto.rotPID(0, 0.5, .5, 5);
                    if (result < 0) nSwitch++;
                    break;
            }
            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.addData("rotation", robot.getWorldRotation());
            telemetry.update();
        }
    }
}
