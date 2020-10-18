package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.BeastBoyHardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.Kisshardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissHardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;

@Autonomous(group = "Calibration")
public class OdometryCalibrator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OldKissBotHArdware robot = new OldKissBotHArdware(this, 0,0,0);
        AutoFunctions auto = new AutoFunctions(robot);
        String choice = "";
        boolean deciding = true;
        //let the user specify a direction to move
        while (!isStarted() && !isStopRequested() && deciding) {
            if (gamepad1.dpad_up) choice = "Forward";
            else if (gamepad1.dpad_down) choice = "Backwards";
            else if (gamepad1.dpad_left) choice = "Left";
            else if (gamepad1.dpad_right) choice = "Right";
            telemetry.addData("choice", choice);
            telemetry.update();
            if (choice != "") deciding = false;
        }
        waitForStart();
        FunctionLibrary.Point destination = null;
        //based off of the choice, tell it to move 2 feet forwards, backwards, left, or right
        double distance = 100;
        if (choice == "Forward") destination = new FunctionLibrary.Point(0, distance);
        else if (choice == "Backwards") destination = new FunctionLibrary.Point(0, -distance);
        else if (choice == "Left") destination = new FunctionLibrary.Point(-distance,0);
        else if (choice == "Right") destination = new FunctionLibrary.Point(distance,0);

        int nSwitch = 0;
        double result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                case 0:
                    //tell the robot to move in the pecified direction
                    result = auto.gotoPosition(destination, 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    //rotate the robot back to it's original rotation
                    result = auto.rotPID(0, 1, 1, 5);
                    if (result < 0) nSwitch++;
                    break;
            }
            //print it's position and rotation
            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.addData("rotation", robot.getWorldRotation());
            telemetry.addData("State", nSwitch);
            telemetry.update();
        }
    }
}
