package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GameChangerOpenCVPipeline;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Libraries.Bases.autoBase;
import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;
import org.firstinspires.ftc.teamcode.worldVariables;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class WobbleAndPark implements autoBase {
    //setup initial variables need for the autonomous program
    private final Telemetry telemetry;
    private final LinearOpMode opMode;
    //initialization function to get robot information and functions
    public WobbleAndPark(LinearOpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;
    }

    //setup hardware map variable
    private GameChangerBotHardware robot;
    //setup for taskhandler that moves the robot in autonomous
    private final taskHandler handler = new taskHandler();
    //setup openCV pipeline variable for detecting the rings
    private GameChangerOpenCVPipeline pipeline;
    private GamechangerAutoValues autoValues;
    private int Alliance = 0;
    private int Auto = 0;
    private int Position = 0;
    private int EndPosition = 0;


    //initialization, take in values needed for doing the right movements
    //initializes openCV pipeline
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        this.Alliance = Alliance;
        this.Auto = Auto;
        this.Position = Position;
        this.EndPosition = EndPosition;
        //setup class for displaying selected autonomous values
        autoValues = new GamechangerAutoValues(telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        //based on picked alliance and starting position, setup the robot's movements
        robot = new GameChangerBotHardware(opMode,0,0,0);
        robot.disableOdometry();
        robot.setRotation(180);
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                //initialize robot hardware map with it's position
                robot.setPosition(-48,-69);
                //move to the side of the rings
                handler.addTask(new baseTasks.move(new Point(-50,-34),180,1,1,5000));
                //move on to the line
                handler.addTask(new baseTasks.move(new Point(-50,0),180,1,1,5000));
            } else { //right
                robot.setPosition(-24,-69);
                handler.addTask(new baseTasks.move(new Point(-18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(-18,0),180,1,1,5000));
            }
        } else { //red
            if (Position == 0) { //right
                robot.setPosition(48,-69);
                handler.addTask(new baseTasks.move(new Point(54,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(54,0),180,1,1,5000));
            } else { //left
                robot.setPosition(24,-69);
                handler.addTask(new baseTasks.move(new Point(18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(18,0),180,1,1,5000));
            }
        }
        robot.enableOdometry();
        //setup openCV pipeline
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //initialize opencv camera factory using the designated webcam
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        //retrieve the configured color ranges from our configuration file
        Scalar[] limits = robot.fromCSV();
        //initialize openCV pipeline with those ranges
        pipeline = new GameChangerOpenCVPipeline(limits[0],limits[1]);
        //tells the webcam to send it's image streams to the openCV pipeline
        webcam.setPipeline(pipeline);
        //tells the webcam to start streaming at a 320x240p resolution
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    //variable for transferring the number of detected rings
    double numRings = 0;
    @Override
    public void init_loop() {
        //get the ratio of the width/height of the rings our pipeline is seeing
        double ratio = pipeline.getRatio();
        //use the ratio to determine the number of rings we see
        if (ratio >= 2.5) {
            numRings = 1;
        } else if(ratio < 2.5 && ratio >= 0.5) {
            numRings = 4;
        } else {
            numRings = 0;
        }
        //output the number of rings
        telemetry.addData("rings: ", numRings);
        telemetry.addData("Ratio: ", ratio);
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
    }
    //from here, we now have the amount of rings we detected from our init loop
    //From this, we define the extra movements based on that data
    @Override
    public void loop_init() {
        //based on the alliance, tell it to move to the corresponding zone based on the number of rings
        if (Alliance == 0) { //blue
            if (numRings == 4) {
                handler.addTask(new baseTasks.move(new Point(-40,30),180,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new Point(-21,16),180,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new Point(-40,-5),180,1,1,5000));
            }
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0.5,200));
            handler.addTask(new baseTasks.servoMovement(robot.wobbleGrab1,0,200));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,1,200));

            //Tell the robot it's given end position that has been selected by the user
            if (EndPosition == 1) { //left
                handler.addTask(new baseTasks.move(new Point(-40,-5),180,1,1,5000));
            } else { //right
                handler.addTask(new baseTasks.move(new Point(-18,-5),180,1,1,5000));
            }


        } else { //red
            if (numRings == 0) {
                handler.addTask(new baseTasks.move(new Point(50,55),180,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new Point(36,31),180,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new Point(50,12),180,1,1,5000));
            }
            if (EndPosition == 0) { //right
                handler.addTask(new baseTasks.move(new Point(50,0),180,1,1,5000));
            } else { //left
                handler.addTask(new baseTasks.move(new Point(18, 0), 180, 1, 1, 5000));
            }
        }
    }
    //run the main loop where the taskhandler will handle the movement based on it's given commands
    @Override
    public void loop() {
        handler.loop(robot);
        telemetry.addData("rings: ", numRings);
        telemetry.addData("curTask: ", handler.curTask);
        telemetry.addData("x: ", robot.getX());
        telemetry.addData("y: ", robot.getY());
        telemetry.update();
    }

    @Override
    public void end() {
        worldVariables.worldRotation = robot.getWorldRotation();
    }
}