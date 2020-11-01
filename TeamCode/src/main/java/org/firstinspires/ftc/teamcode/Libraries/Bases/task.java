package org.firstinspires.ftc.teamcode.Libraries.Bases;

//base interface used to make tasks that the robot can do in autonomous
public interface task {
    //function ran to tell the task it's about time to start running
    public void init();
    //main loop, ran every iteration and it takes a number output
    //this number tells the taskhandler when it should move on to the next task
    public int loop(RobotConstructor robot);
}
