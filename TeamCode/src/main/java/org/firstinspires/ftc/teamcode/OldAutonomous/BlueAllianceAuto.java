package org.firstinspires.ftc.teamcode.OldAutonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

@Autonomous
@Disabled
public class BlueAllianceAuto extends LinearOpMode {
    private final double mmPerInch = 25.4;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize our hardware and backend stuff
        D1V4hardware robot = new D1V4hardware(this,0);
        //initialize our auto functions class
        AutoFunctions auto = new AutoFunctions(robot);

        //initalize a control class for every the inout, updown, and openclose motors
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcUpDown1, robot.dcUpDown2);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(5,robot.dcOpenClose);
        //set the offsets from the side and front to the center
        double leftRightCenterOffset = 8.5;
        double backFrontCenterOffset = 7;

        //tell the user that vuforia is initalizing
        telemetry.addData("Startup:", "initializing vuforia");
        telemetry.update();
        //initalize vuforia
        robot.initVuforia(hardwareMap);
        //tell the user that vuforia is done initializing
        telemetry.addData("Startup:", "Ready to go!");
        telemetry.update();
        //wait for start
        waitForStart();
        //activate the vuforia trackables
        robot.SkystoneTrackables.activate();
        //set the current position
        robot.setPosition(-(72-backFrontCenterOffset),-(24-leftRightCenterOffset));
        //set the current rotation
        robot.setRotation(90);
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
            switch(nSwitch) {
                //removed this case so I just made it go on
                case 0:
                    nSwitch++;
                case 1:
                    //move the inout out of the way
                    result = inoutControl.move_using_encoder(405,0.5,5,20,false);
                    if (result < 0 ) nSwitch++;
                    break;
                case 2:
                    //open the grabber more to make it easier to grab a block
                    result = openCloseControl.move_using_encoder(-1500,1,4,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    //head to the point in between the first two skystones
                    destination = new FunctionLibrary.Point(-(36+robot.CameraForwardDisplacement)-6, -(30+robot.CameraLeftDisplacement));
                    result = auto.gotoPosition(destination,1,1,90);
                    if (result < 0) {
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
                case 4:
                    //look for the skystone
                    robot.updateVuforia();
                    //if we find it, set the current position as the our current offset from it plus our current position
                    if(robot.VuMarkPositions.containsKey("Stone Target")) {
                        resetStartTime();
                        VectorF stonePos = robot.VuMarkPositions.get("Stone Target");
                        SkystoneTarget = new FunctionLibrary.Point((-stonePos.get(0)/mmPerInch)+robot.getX(),-(stonePos.get(1)/mmPerInch)+robot.getY());
                        Log.d("BothAutoVuforia", "x: " + SkystoneTarget.x + "y: " + SkystoneTarget.y);
                        nSwitch++;
                    } else if (getRuntime() > 2) { //if it's been more than two seconds assume it's the third stone
                        SkystoneTarget = new FunctionLibrary.Point(-36,-42);
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
                case 5:
                    //line up with the stone
                    destination = new FunctionLibrary.Point(-(36+robot.CameraForwardDisplacement)-6, SkystoneTarget.y+2);
                    result = auto.gotoPosition(destination,1,1,90);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    //ram into it
                    destination = new FunctionLibrary.Point(SkystoneTarget.x-2, SkystoneTarget.y+2);
                    result = auto.gotoPosition(destination,0.5,1,90);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    //close the grabber in order to grab it
                    result = openCloseControl.move_using_encoder(9500, 1, 6,10,true);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    //move the inout further so it doesn't interfere with driving
                    result = inoutControl.move_using_encoder(500,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    //move towards the wall
                    destination = new FunctionLibrary.Point(-54,SkystoneTarget.y);
                    result = auto.gotoPosition(destination,1,1,90);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    //rotate to 0 degrees and drive underneath the bridge
                    destination = new FunctionLibrary.Point(-54,5);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    //move the lift up
                    result = upDownControl.move_using_encoder(8000,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    //move towards the foundation
                    destination = new FunctionLibrary.Point(-20,10);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 13:
                    //line up with the foundation
                    destination = new FunctionLibrary.Point(-25,20);
                    result = auto.gotoPosition(destination,1,10,0);
                    if (result < 0) nSwitch++;
                    break;
                case 14:
                    //drop the block
                    result = openCloseControl.move_using_encoder(-1000, 1, 6,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 15:
                    //move in front of the bridge
                    destination = new FunctionLibrary.Point(-54,5);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 16:
                    //retract the lift to make the robot fit undearneath the robot
                    result = upDownControl.move_encoder_to_position(100,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 17:
                    //park under the bridge
                    destination = new FunctionLibrary.Point(-54,-5);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;



            }
        }
    }
}