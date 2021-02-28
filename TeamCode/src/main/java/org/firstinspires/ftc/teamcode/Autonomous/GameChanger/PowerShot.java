package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.Bases.autoBase;
import org.firstinspires.ftc.teamcode.Libraries.GameChanger.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;


//Class inherits from autoBase and extends it to do the actual autonomous work
public class PowerShot implements autoBase {
    //initialize variables needed for the program to run
    private final LinearOpMode opMode;
    //initialization function to get the opmode so we can access the robot information
    public PowerShot(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    //Setup robot hardwaremap class
    private GameChangerBotHardware robot;
    //Make a new task handler for autonomous movement
    private taskHandler handler = new taskHandler();

    //initialization, takes the values and sets up initial movements
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        GamechangerAutoValues autoValues = new GamechangerAutoValues(opMode.telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        opMode.telemetry.update();
        robot = new GameChangerBotHardware(opMode,0,0,180);
        robot.disableOdometry();
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.deflector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.CAM.setPosition(0.67);
        //based on alliance and starting position pre-program the robot's movements
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                //initialize the hardware map with the robots current position
                robot.setPosition(-48,-71);
                //move forward and to the side of the rings
                handler.addTask(new baseTasks.move(new Point(-54,-34),180,1,1,5000));
                //park on the line, same thing with the rest of the functions
                handler.addTask(new baseTasks.move(new Point(-54,-14),180,1,1,5000));
            } else { //right
                robot = new GameChangerBotHardware(opMode,-24,-71,180);
                handler.addTask(new baseTasks.move(new Point(-18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(-18,-14),180,1,1,5000));
            }
            handler.addTask(new baseTasks.move(new Point(-30,-13),0,0.5,0.5,5000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,500,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(-25,-13),0,0.5,0.5,5000));
            handler.addTask(new baseTasks.wait(500));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,1000,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(-13,-13),0,0.5,0.5,5000));
            handler.addTask(new baseTasks.wait(500));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,1500,1,10,2000));


        } else { //red
            if (Position == 0) { //right
                robot.setPosition(48,-71);
                handler.addTask(new baseTasks.move(new Point(54,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(54,-9),180,1,1,5000));
            } else { //left
                robot = new GameChangerBotHardware(opMode,24,-71,180);
                handler.addTask(new baseTasks.move(new Point(18,-34),180,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(18,-9),180,1,1,5000));
            }
            handler.addTask(new baseTasks.move(new Point(14,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,500,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(11,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,1000,1,10,2000));
            handler.addTask(new baseTasks.move(new Point(8,3),180,0.5,0.5,5000));
            handler.addTask(new baseTasks.motorMovement(robot.magazine,1500,1,10,2000));
        }
        robot.enableOdometry();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop_init() {
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        robot.shooter.setPower(-0.72);
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
