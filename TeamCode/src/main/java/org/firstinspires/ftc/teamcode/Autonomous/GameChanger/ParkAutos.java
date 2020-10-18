package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.GamechangerAutoValues;
import org.firstinspires.ftc.teamcode.Functions.autoBase;
import org.firstinspires.ftc.teamcode.Functions.baseTasks;
import org.firstinspires.ftc.teamcode.Functions.taskHandler;
import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;

public class ParkAutos implements autoBase {
    private final LinearOpMode opMode;
    public ParkAutos(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    private OldKissBotHArdware robot;
    private taskHandler handler = new taskHandler();

    @Override
    public void init(int Alliance, int Auto, int Position, int EndPosition) {
        GamechangerAutoValues autoValues = new GamechangerAutoValues(opMode.telemetry);
        //print out the values read in text form
        autoValues.translateValues(Alliance, Auto, Position, EndPosition);
        opMode.telemetry.update();
        if (Alliance == 0) { //blue
            if (Position == 1) { //left
                robot = new OldKissBotHArdware(opMode,-48,-71,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-54,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-54,0),0,1,1,5000));
            } else { //right
                robot = new OldKissBotHArdware(opMode,-24,-71,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(-18,0),0,1,1,5000));
            }
        } else { //red
            if (Position == 0) { //right
                robot = new OldKissBotHArdware(opMode,48,-71,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(54,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(54,0),0,1,1,5000));
            } else { //left
                robot = new OldKissBotHArdware(opMode,24,-71,0);
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(18,-34),0,1,1,5000));
                handler.addTask(new baseTasks.move(new FunctionLibrary.Point(18,0),0,1,1,5000));
            }
        }
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop_init() {

    }
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
