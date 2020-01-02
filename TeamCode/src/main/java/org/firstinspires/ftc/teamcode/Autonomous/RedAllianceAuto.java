package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

@Autonomous
public class RedAllianceAuto extends LinearOpMode {
    private final double mmPerInch = 25.4;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize the robot hardware and odometry
        D1V4hardware robot = new D1V4hardware(this,0);
        //initialize the auto functions class
        AutoFunctions auto = new AutoFunctions(robot);

        //initalize the motor controller classes for the inout, lift, and openclose motors
        FunctionLibrary.motorMovement inoutControl = new FunctionLibrary.motorMovement(100,robot.dcInOut);
        FunctionLibrary.motorMovement upDownControl = new FunctionLibrary.motorMovement(100, robot.dcUpDown1, robot.dcUpDown2);
        FunctionLibrary.motorMovement openCloseControl = new FunctionLibrary.motorMovement(3,robot.dcOpenClose);
        //set the offsets from the side and front to the center
        double leftRightCenterOffset = 8.5;
        double backFrontCenterOffset = 7;
        //tell the user vuforia is initializing
        telemetry.addData("Startup:", "Vuforia is initializing!");
        telemetry.update();
        //initialize vuforia
        robot.initVuforia(hardwareMap);
        //tell the user that vuforia has been initialized
        telemetry.addData("Startup:","Ready for startup!");
        telemetry.update();
        //waitforstart
        waitForStart();
        //activate the trackables
        robot.SkystoneTrackables.activate();
        //set our starting position
        robot.setPosition((72-backFrontCenterOffset),-(24-leftRightCenterOffset));
        //set our starting rotation
        robot.setRotation(-90);
        //create a point to store the skystone position when we find it
        FunctionLibrary.Point SkystoneTarget = null;
        //setup the initial state
        int nSwitch = 0;
        while (opModeIsActive()) {
            //point used for movement
            FunctionLibrary.Point destination = null;
            //used to find the result of autonmous iterations
            double result = 0;
            //state machine
            switch(nSwitch) {
                case 0: //took this out so I just have it go to the next state
                    nSwitch++;
                case 1:
                    //move the inout out a bit to allow movement
                    result = inoutControl.move_using_encoder(600,0.5,5,20,false);
                    if (result < 0 ) nSwitch++;
                    break;
                case 2:
                    //open the grabber to make it easier to grab a block
                    result = openCloseControl.move_using_encoder(-1500,1,4,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 3:
                    //drive towards the point in between the first two blocks
                    destination = new FunctionLibrary.Point((36+robot.CameraForwardDisplacement)+6, -(30+robot.CameraLeftDisplacement)+4);
                    result = auto.gotoPosition(destination,1,1,-90);
                    if (result < 0) {
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
                case 4:
                    //update vuforia
                    robot.updateVuforia();
                    //if the skystone is found, head towards it
                    //otherwise, assume it's the third and head towards that
                    if(robot.VuMarkPositions.containsKey("Stone Target")) {
                        resetStartTime();
                        VectorF stonePos = robot.VuMarkPositions.get("Stone Target");
                        SkystoneTarget = new FunctionLibrary.Point((-(stonePos.get(0)/mmPerInch))+robot.getX(),(stonePos.get(1)/mmPerInch)+robot.getY());
                        Log.d("BothAutoVuforia", "x: " + SkystoneTarget.x + "y: " + SkystoneTarget.y);
                        nSwitch++;
                    } else if (getRuntime() > 2) {
                        SkystoneTarget = new FunctionLibrary.Point(36,-46);
                        resetStartTime();
                        nSwitch++;
                    }
                    break;
                case 5:
                    //line up with the skystone
                    destination = new FunctionLibrary.Point((36+robot.CameraForwardDisplacement)+6, SkystoneTarget.y-2);
                    result = auto.gotoPosition(destination,1,1,-90);
                    if (result < 0) nSwitch++;
                    break;
                case 6:
                    //ram into the skystone
                    destination = new FunctionLibrary.Point(SkystoneTarget.x+2, SkystoneTarget.y-2);
                    result = auto.gotoPosition(destination,0.5,1,-90);
                    if (result < 0) nSwitch++;
                    break;
                case 7:
                    //close the grabber on it
                    result = openCloseControl.move_using_encoder(10500, 1, 6,10,true);
                    if (result < 0) nSwitch++;
                    break;
                case 8:
                    //move the inout out in order to allow for movement
                    result = inoutControl.move_using_encoder(500,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 9:
                    //head towards the wall
                    destination = new FunctionLibrary.Point(54,SkystoneTarget.y);
                    result = auto.gotoPosition(destination,1,1,-90);
                    if (result < 0) nSwitch++;
                    break;
                case 10:
                    //drive underneath the bridge while rotating to face it
                    destination = new FunctionLibrary.Point(54,5);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 11:
                    //move the updown up to drop the block on the foundation
                    result = upDownControl.move_using_encoder(8000,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 12:
                    //move towards the foundation
                    destination = new FunctionLibrary.Point(20,10);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 13:
                    //line up with the foundation
                    destination = new FunctionLibrary.Point(25,20);
                    result = auto.gotoPosition(destination,1,10,0);
                    if (result < 0) nSwitch++;
                    break;
                case 14:
                    //drop the block on the foundation
                    result = openCloseControl.move_using_encoder(-1000, 1, 6,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 15:
                    //move in front of the skybridge
                    destination = new FunctionLibrary.Point(54,5);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;
                case 16:
                    //bring the updown down to allow us to go under the bridge
                    result = upDownControl.move_encoder_to_position(100,1,5,10,false);
                    if (result < 0) nSwitch++;
                    break;
                case 17:
                    //park underneath the bridge
                    destination = new FunctionLibrary.Point(54,-10);
                    result = auto.gotoPosition(destination,1,1,0);
                    if (result < 0) nSwitch++;
                    break;



            }
        }
    }
}