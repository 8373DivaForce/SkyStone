package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Functions.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Functions.autoBase;
import org.firstinspires.ftc.teamcode.Functions.baseTasks;
import org.firstinspires.ftc.teamcode.Functions.taskHandler;
import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;


//Class inherits from autoBase and extends it to do the actual autonomous work
public class ParkAutos implements autoBase {
    //initialize variables needed for the program to run
    private final LinearOpMode opMode;
    //initialization function to get the opmode so we can access the robot information
    public ParkAutos(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    //Setup robot hardwaremap class
    private OldKissBotHArdware robot;
    //Make a new task handler for autonomous movement
    private taskHandler handler = new taskHandler();

    //initialization, takes the values and sets up initial movements
    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        GamechangerAutoValues autoValues = new GamechangerAutoValues(opMode.telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        opMode.telemetry.update();
        //based on alliance and starting position pre-program the robot's movements
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                //initialize the hardware map with the robots current position
                robot = new OldKissBotHArdware(opMode,-48,-71,0);
                //move forward and to the side of the rings
                handler.addTask(new baseTasks.move(new Point(-54,-34),0,1,1,5000));
                //park on the line, same thing with the rest of the functions
                handler.addTask(new baseTasks.move(new Point(-54,0),0,1,1,5000));
            } else { //right
                robot = new OldKissBotHArdware(opMode,-24,-71,0);
                handler.addTask(new baseTasks.move(new Point(-18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(-18,0),0,1,1,5000));
            }
        } else { //red
            if (Position == 0) { //right
                robot = new OldKissBotHArdware(opMode,48,-71,0);
                handler.addTask(new baseTasks.move(new Point(54,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(54,0),0,1,1,5000));
            } else { //left
                robot = new OldKissBotHArdware(opMode,24,-71,0);
                handler.addTask(new baseTasks.move(new Point(18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new Point(18,0),0,1,1,5000));
            }
        }
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop_init() {

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