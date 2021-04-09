package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.Bases.autoBase;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GameChangerOpenCVPipeline;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


//Class inherits from autoBase and extends it to do the actual autonomous work
public class HighWobble implements autoBase {
    //initialize variables needed for the program to run
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    //initialization function to get the opmode so we can access the robot information
    public HighWobble(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }
    //Setup robot hardwaremap class
    private GameChangerBotHardware robot;
    //Make a new task handler for autonomous movement
    private taskHandler handler = new taskHandler();
    GameChangerOpenCVPipeline pipeline;
    GamechangerAutoValues autoValues;
    int Alliance = 0;
    int Auto = 0;
    int Position = 0;
    int EndPosition = 0;
    //initialization, takes the values and sets up initial movements
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        autoValues = new GamechangerAutoValues(opMode.telemetry);
        this.Alliance = Alliance;
        this.Auto = Auto;
        this.Position = Position;
        this.EndPosition = EndPosition;
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        opMode.telemetry.update();
        robot = new GameChangerBotHardware(opMode,0,0,180);
        robot.disableOdometry();
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.deflector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.CAM.setPosition(1);
        //based on alliance and starting position pre-program the robot's movements
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                //initialize the hardware map with the robots current position
                robot.setPosition(-48,-71);
                //move forward and to the side of the rings
                handler.addTask(new baseTasks.move(new Point(-53,-32),180,1,1,5000));
                //move on to the line as an intermediary point
                handler.addTask(new baseTasks.move(new Point(-53,-14),180,1,1,5000));
            } else { //right
                robot.setPosition(-24,-71);
                handler.addTask(new baseTasks.move(new Point(-18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(-18,-14),180,1,1,5000));
            }
            //moves in front of power shot 1, makes sure it is rotated correctly, then shoots.
            handler.addTask(new baseTasks.move(new Point(-43,-7),0, 0.5,1,5000));
            handler.addTask(new baseTasks.rotate(0,0.25,2,2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,300,1,10,2000));
            //moves in front of power shot 2, waits to make sure the shooter is up to speed, then shoots
            //handler.addTask(new baseTasks.move(new Point(-22,-17),0,0.5,1,5000));
            handler.addTask(new baseTasks.wait(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,800,1,10,2000));
            //moves in front of power shot 3, waits to make sure the shooter is up to speed, then shoots
            //handler.addTask(new baseTasks.move(new Point(-18,-17),0,0.5,1,5000));
            handler.addTask(new baseTasks.wait(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,1600,1,10,2000));


        } else { //red
            if (Position == 0) { //right
                robot.setPosition(48,-71);
                handler.addTask(new baseTasks.move(new Point(54,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(54,-9),180,1,1,5000));
            } else { //left
                robot.setPosition(24,-71);
                handler.addTask(new baseTasks.move(new Point(18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(18,-9),180,1,1,5000));
            }
            handler.addTask(new baseTasks.move(new Point(14,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.rotate(0,0.5,1,1000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,500,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(11,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.wait(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,4000,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(8,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.wait(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,4500,1,10,2000));
        }
        //enables the odometry after intializing it for autonomous
        robot.enableOdometry();
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
    //varible to transfer the number of rings detected across run states
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

    @Override
    public void loop_init() {
        //initialize the intake and shooter
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        robot.shooter.setPower(-0.65);
        if (Alliance == 0) { //blue
            //based on the number of rings, program the position for the zone the robot needs to go to
            if (numRings == 4) {
                handler.addTask(new baseTasks.move(new Point(-40,48),180,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new Point(-21,24),180,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new Point(-40,0),180,1,1,5000));
            }
            //bring the wobble goal down, release it, and then bring it back up
            handler.addTask(new baseTasks.rotate(180,0.5,3,1000));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0.5,400));
            handler.addTask(new baseTasks.servoMovement(robot.wobbleGrab,0,400));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,1,400));

            //Tell the robot it's given end position that has been selected by the user
            if (EndPosition == 1) { //left
                handler.addTask(new baseTasks.move(new Point(-40,6),180,1,1,5000));
            } else { //right
                handler.addTask(new baseTasks.move(new Point(-18,4),180,1,1,5000,24));

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
    //for the main loop, just keep running the task handler and let it make the robot move
    //also output debugging information
    @Override
    public void loop() {
        handler.loop(robot);
        opMode.telemetry.addData("Current Task:", handler.curTask);
        opMode.telemetry.update();
    }

    @Override
    public void end() {

    }
}
