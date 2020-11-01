package org.firstinspires.ftc.teamcode.Libraries.functions;

import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.Bases.task;

import java.util.ArrayList;

public class taskHandler {
    //initialize array of tasks the program needs to run
    private ArrayList<task> tasks = new ArrayList<>();
    //function for adding tasks to the task handler
    public void addTask(task Task) {
        tasks.add(Task);
    }
    //Variable for keeping track of the task it is currently running
    public int curTask = 0;
    //variable so it knows whether or not to run the tasks' initialization or it's main loop
    boolean started = false;
    public int loop(RobotConstructor robot) {
        if (curTask < tasks.size()) {
            //if the task hasn't been started yet, initialize it
            if (!started) {
                tasks.get(curTask).init();
                started = true;
            }
            //run one iteration of the task's main loop and retrieve it's output
            int output = tasks.get(curTask).loop(robot);
            //if the output is less than 0, then it is either errored or done, so we will move on to the next task
            if (output < 0) {
                curTask++;
                started = false;
            }
        }
        return curTask;
    }
}
