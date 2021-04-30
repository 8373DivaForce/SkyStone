package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.Bases.autoBase;
import org.firstinspires.ftc.teamcode.Libraries.Bases.task;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GameChangerOpenCVPipeline;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Libraries.functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;
import org.firstinspires.ftc.teamcode.robotConstants;
import org.firstinspires.ftc.teamcode.worldVariables;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


//Class inherits from autoBase and extends it to do the actual autonomous work
public class HighWobblePlus implements autoBase {

    //initialize variables needed for the program to run
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    //initialization function to get the opmode so we can access the robot information
    public HighWobblePlus(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }
    //Setup robot hardwaremap class
    private static GameChangerBotHardware robot;
    //Make a new task handler for autonomous movement
    private taskHandler handler = new taskHandler();
    GameChangerOpenCVPipeline pipeline;
    GamechangerAutoValues autoValues;
    int Alliance = 0;
    int Auto = 0;
    int Position = 0;
    int EndPosition = 0;
    public static class waitForSpeedUp implements task {
        //initial variables for auto task
        private final double timeOut;
        private double startTime = 0;
        public waitForSpeedUp(double timeOut) {
            this.timeOut = timeOut;
        }
        @Override
        public void init() {
            //set the start time for the timeout and set the motors to run using encoders
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //check if the program has ran for too long, if so, terminate it
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;
            //iterate through each motor
            if (Math.abs(((DcMotorEx) HighWobblePlus.robot.shooter).getVelocity()) > robotConstants.shooterSpeed-120) return -1;
            //return that the program is still running if it has gotten to this point
            return 1;
        }
    }
    public static class stopShooter implements task {
        //initial variables for auto task
        @Override
        public void init() {
            //set the start time for the timeout and set the motors to run using encoders
            HighWobblePlus.robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            HighWobblePlus.robot.shooter.setPower(0);
            HighWobblePlus.robot.shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            HighWobblePlus.robot.intakeRD.setPower(0);
            HighWobblePlus.robot.deflector.setPower(0);
            HighWobblePlus.robot.magazine.setPower(0);
        }

        @Override
        public int loop(RobotConstructor robot) {
            return -1;
        }
    }
    public static class pickUpNShootRings implements task {
        //initial variables for auto task
        private final double timeOut;
        private final double angle;
        private final double approachSpeed;
        private final double leaveSpeed;
        private final double intakeVoltage;
        private double startTime = 0;

        public pickUpNShootRings(double timeOut, double angle, double approachSpeed, double leaveSpeed, double intakeVoltage) {
            this.timeOut = timeOut;
            this.angle = angle;
            this.approachSpeed = approachSpeed;
            this.leaveSpeed = leaveSpeed;
            this.intakeVoltage = intakeVoltage;
        }
        @Override
        public void init() {
            //set the start time for the timeout and set the motors to run using encoders
            startTime = System.currentTimeMillis();
        }
        private AutoFunctions auto = null;
        double previousSpeed = 0;
        double curRing = 0;
        //state 0 = backing up, state 1 = waiting, state 2 = movingAway
        int state = 0;
        Point forwardPoint = null;
        @Override
        public int loop(RobotConstructor robot) {
            //check if the program has ran for too long, if so, terminate it
            if (auto == null) auto = new AutoFunctions(robot);
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;

            int backState = 0;
            if (state == 0) {
                backState = auto.gotoPosition(new Point(-9,-45),approachSpeed,1,angle);
            }
            else if (state == 1){
                if (Math.abs(((DcMotorEx) HighWobblePlus.robot.shooter).getVelocity()-previousSpeed) >= 80) state = 0;
            } else {
                int res = auto.gotoPosition(forwardPoint, leaveSpeed, 1, angle);
                if (res < 0) {
                    if (curRing == 0) {
                        state = 1;
                    } else {
                        state = 0;
                    }
                }
            }

            if (((DcMotorEx)HighWobblePlus.robot.intakeRD).getCurrent(CurrentUnit.AMPS) > 1.2) {
                previousSpeed = ((DcMotorEx)HighWobblePlus.robot.shooter).getVelocity();
                Point roboPos = robot.getPosition();
                forwardPoint = new Point(roboPos.x, roboPos.y+3);
                state = 2;
            }

            if (backState < 0) return -1;
            //return that the program is still running if it has gotten to this point
            return 1;
        }
    }
    //initialization, takes the values and sets up initial movements
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        autoValues = new GamechangerAutoValues(telemetry);
        this.Alliance = Alliance;
        this.Auto = Auto;
        this.Position = Position;
        this.EndPosition = EndPosition;
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        telemetry.update();
        robot = new GameChangerBotHardware(opMode,0,0,180);
        robot.disableOdometry();
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.deflector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.CAM.setPosition(0.45);
        //based on alliance and starting position pre-program the robot's movements
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                //initialize the hardware map with the robots current position
                robot.setPosition(-48,-71);
                //move forward and to the side of the rings
                handler.addTask(new baseTasks.move(new Point(-53,-32),180,1,3,5000));
                //move on to the line as an intermediary point
                handler.addTask(new baseTasks.move(new Point(-53,-14),180,1,3,5000));
            } else { //right
                robot.setPosition(-24,-71);
                handler.addTask(new baseTasks.move(new Point(-18,-34),180,1,3,5000));
                handler.addTask(new baseTasks.move(new Point(-18,-14),180,1,3,5000));
            }
            //moves in front of power shot 1, makes sure it is rotated correctly, then shoots.
            handler.addTask(new baseTasks.move(new Point(-21,-15),0, 0.85,1,5000));
            handler.addTask(new baseTasks.rotate(0,0.25,2,2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,300,1,10,2000));
            //moves in front of power shot 2, waits to make sure the shooter is up to speed, then shoots
            //handler.addTask(new baseTasks.move(new Point(-22,-17),0,0.5,1,5000));
            handler.addTask(new waitForSpeedUp(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,800,1,10,2000));
            //moves in front of power shot 3, waits to make sure the shooter is up to speed, then shoots
            //handler.addTask(new baseTasks.move(new Point(-18,-17),0,0.5,1,5000));
            handler.addTask(new waitForSpeedUp(2000));
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
            handler.addTask(new waitForSpeedUp(2000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,4000,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(8,3),180,0.5,0.5,5000));
            handler.addTask(new waitForSpeedUp(2000));
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
        opMode.resetStartTime();
        //initialize the intake and shooter
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        ((DcMotorEx)robot.shooter).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, robotConstants.pidfValsShooter);
        ((DcMotorEx)robot.shooter).setVelocity(robotConstants.shooterSpeed);
        if (Alliance == 0) { //blue
            //based on the number of rings, program the position for the zone the robot needs to go to
            if (numRings == 4) {
                handler.addTask(new baseTasks.move(new Point(-40,36),180,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new Point(-21,24),180,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new Point(-40,0),180,1,1,5000));
            }
                //bring the wobble goal down, release it, and then bring it back up
            handler.addTask(new baseTasks.rotate(180,0.25,5,1000));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0.5,200));
            handler.addTask(new baseTasks.servoMovement(new Servo[]{robot.wobbleGrab1, robot.wobbleGrab2},new double[]{0,0},300));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0,300));

            handler.addTask(new baseTasks.rotate(0,0.8,10,1000));
            handler.addTask(new baseTasks.move(new Point(-11,-25),0, .9,1,5000));
            if (numRings > 0) {
                handler.addTask(new baseTasks.rotate(-6, 0.2, 2, 1000));
                handler.addTask(new baseTasks.setMotorPower(1, robot.magazine));
                handler.addTask(new baseTasks.servoMovement(robot.CAM, .6, 200));


                handler.addTask(new baseTasks.move(new Point(-11, -30), -6, 0.5, 1, 5000));
                handler.addTask(new baseTasks.wait(500));
                if (numRings == 4) {
                    handler.addTask(new baseTasks.move(new Point(-11, -37), -6, 0.5, 1, 5000));
                    handler.addTask(new baseTasks.move(new Point(-11, -33), -6, 0.7, 1, 5000));

                    handler.addTask(new baseTasks.servoMovement(robot.CAM, .55, 200));

                    handler.addTask(new baseTasks.move(new Point(-11, -45), -6, 0.5, 1, 5000));
                    handler.addTask(new baseTasks.move(new Point(-11, -38), -6, 0.7, 1, 5000));

                    handler.addTask(new baseTasks.move(new Point(-11, -48), -6, 0.5, 1, 5000));


                    //handler.addTask(new pickUpNShootRings(20000,-7,0.4,0.6,1));
                    handler.addTask(new baseTasks.wait(3000));
                }
            }
            handler.addTask(new baseTasks.move(new Point(-12,-47), 0, 0.6, 1, 3000));

            handler.addTask(new baseTasks.move(new Point(-12,-55), 0, 0.6, 1, 3000));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0.5,300));
            handler.addTask(new baseTasks.move(new Point(-9.5,-55), 0, 0.4, 1, 1000));
            handler.addTask(new baseTasks.servoMovement(new Servo[]{robot.wobbleGrab1, robot.wobbleGrab2},new double[]{1,1},300));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0,100));
            handler.addTask(new stopShooter());
            if (numRings == 4) {
                handler.addTask(new baseTasks.move(new Point(-40,36),180,1,1,5000));
            } else if (numRings == 1) {
                handler.addTask(new baseTasks.move(new Point(-21,24),180,1,1,5000));
            } else {
                handler.addTask(new baseTasks.move(new Point(-40,0),180,1,1,5000));
            }
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0.5,200));
            handler.addTask(new baseTasks.servoMovement(new Servo[]{robot.wobbleGrab1, robot.wobbleGrab2},new double[]{0,0},300));
            handler.addTask(new baseTasks.servoMovement(robot.wobblePivot,0,200));
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
        telemetry.addData("Current Task:", handler.curTask);
        telemetry.addData("Velocity", ((DcMotorEx)robot.shooter).getVelocity());
        telemetry.addData("IntakeAmps", ((DcMotorEx)robot.intakeRD).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RunTime", opMode.getRuntime());
        telemetry.update();
        if (opMode.getRuntime() >= 30) opMode.stop();
    }

    @Override
    public void end() {
        worldVariables.worldRotation = robot.getWorldRotation();
    }
}
