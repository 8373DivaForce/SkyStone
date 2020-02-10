package org.firstinspires.ftc.teamcode.OldAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

@Autonomous
@Disabled
public class BridgeRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize our hardware and backend stuff
        D1V4hardware robot = new D1V4hardware(this,0);
        //initialize our auto functions class
        AutoFunctions auto = new AutoFunctions(robot);

        //initalize a control class for every the inout, updown, and openclose motors
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcUpDown1, robot.dcUpDown2);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(10,robot.dcOpenClose);
        //set the offsets from the side and front to the center
        double leftRightCenterOffset = 8.5;
        double backFrontCenterOffset = 7;
        telemetry.addData("Startup", "Ready for start!");
        telemetry.update();
        //wait for start
        waitForStart();
        //set the current position
        robot.setPosition((72-backFrontCenterOffset),-(24-leftRightCenterOffset));
        //set the current rotation
        robot.setRotation(-90);
        //create an object for when we find the skystone
        FunctionLibrary.Point SkystoneTarget = null;
        //setup the initial switch state
        int nSwitch = 0;
        while (opModeIsActive()) {
            //define a destiantion point which is used in several states
            FunctionLibrary.Point destination = null;
            //define the result which is used in every state
            double result = 0;
            //state machine
            switch (nSwitch) {
                //removed this case so I just made it go on
                case 0:
                    nSwitch++;
                case 1:
                    //move the inout out of the way
                    result = inoutControl.move_using_encoder(405, 0.5, 5, 20, false);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    //open the grabber more to make it easier to grab a block
                    result = openCloseControl.move_using_encoder(-1531, 1, 4, 10, true);
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    //head to the point in between the first two skystones
                    destination = new FunctionLibrary.Point((72-backFrontCenterOffset)-20, 0);
                    result = auto.gotoPosition(destination, 1, 1, -90);
                    if (result < 0) {
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
            }
        }
    }
}
