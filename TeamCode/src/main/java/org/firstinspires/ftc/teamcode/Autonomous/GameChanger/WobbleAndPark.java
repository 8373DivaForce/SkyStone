package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.GameChangerOpenCVPipeline;
import org.firstinspires.ftc.teamcode.Functions.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Functions.autoBase;
import org.firstinspires.ftc.teamcode.Functions.baseTasks;
import org.firstinspires.ftc.teamcode.Functions.taskHandler;
import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class WobbleAndPark implements autoBase {
    private final Telemetry telemetry;
    private final LinearOpMode opMode;
    public WobbleAndPark(LinearOpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;
    }
    private OldKissBotHArdware robot;
    private final taskHandler handler = new taskHandler();
    private GameChangerOpenCVPipeline pipeline;
    private GamechangerAutoValues autoValues;
    private int Alliance = 0;
    private int Auto = 0;
    private int Position = 0;
    private int EndPosition = 0;
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        this.Alliance = Alliance;
        this.Auto = Auto;
        this.Position = Position;
        this.EndPosition = EndPosition;
        autoValues = new GamechangerAutoValues(telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                robot = new OldKissBotHArdware(opMode,-48,-69,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-54,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-54,0),0,1,1,5000));
            } else { //right
                robot = new OldKissBotHArdware(opMode,-24,-69,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-18,0),0,1,1,5000));
            }
        } else { //red
            if (Position == 0) { //right
                robot = new OldKissBotHArdware(opMode,48,-69,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(54,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(54,0),0,1,1,5000));
            } else { //left
                robot = new OldKissBotHArdware(opMode,24,-69,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(18,0),0,1,1,5000));
            }
        }
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        //intiailize opencv camera factory using the designated webcam
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();//open camera
        //initialize our detector pipeline
        Scalar[] limits = robot.fromCSV();
        double[] mins = limits[0].val;
        double[] maxes = limits[1].val;
        pipeline = new GameChangerOpenCVPipeline(limits[0],limits[1]);
        webcam.setPipeline(pipeline);//different stages
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC
    }

    double numRings = 0;
    @Override
    public void init_loop() {
        double ratio = pipeline.getRatio();
        if (ratio >= 2.5) {
            numRings = 1;
        } else if(ratio < 2.5 && ratio >= 0.5) {
            numRings = 4;
        } else {
            numRings = 0;
        }
        telemetry.addData("rings: ", numRings);
        telemetry.addData("Ratio: ", ratio);
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
    }

    @Override
    public void loop_init() {
        if (Alliance == 0) { //blue
            if (numRings == 0) {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-55,55),0,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-36,31),0,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-55,12),0,1,1,5000));
            }
            if (EndPosition == 1) { //left
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-54,0),0,1,1,5000));
            } else { //right
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-18,0),0,1,1,5000));
            }
        } else { //red
            if (numRings == 0) {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(55,55),0,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(36,31),0,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(55,12),0,1,1,5000));
            }
            if (EndPosition == 0) { //right
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(54,0),0,1,1,5000));
            } else { //left
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(18, 0), 0, 1, 1, 5000));
            }
        }
    }
    @Override
    public void loop() {
        handler.loop(robot);
        telemetry.addData("curTask: ", handler.curTask);
    }

    @Override
    public void end() {

    }
}