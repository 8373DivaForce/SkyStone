package org.firstinspires.ftc.teamcode.Functions;

import org.firstinspires.ftc.teamcode.Functions.RobotConstructor;
import org.firstinspires.ftc.teamcode.Functions.task;

import java.util.ArrayList;

public class taskHandler {
    private ArrayList<task> tasks = new ArrayList<>();
    public void addTask(task Task) {
        tasks.add(Task);
    }
   public int curTask = 0;
    boolean started = false;
    public int loop(RobotConstructor robot) {
        if (curTask < tasks.size()) {
            if (!started) {
                tasks.get(curTask).init();
                started = true;
            }
            int output = tasks.get(curTask).loop(robot);
            if (output < 0) {
                curTask++;
                started = false;
            }
        }
        return curTask;
    }
}
