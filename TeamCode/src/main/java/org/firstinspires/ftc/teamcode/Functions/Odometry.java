package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

//class for running the odometry in a separate thread
public class Odometry implements Runnable {
    //takes a robotconstructor in order to call the updateOdometry function
    private final RobotConstructor robot;
    //takes a LinearOpMode to automaticly shut down when the opMode does
    private final LinearOpMode opMode;
    ElapsedTime localTime = new ElapsedTime();
    //constructor class
    DataLogger Dl = null;
    private final String robotName;
    public Odometry(RobotConstructor robot, LinearOpMode opMode, String name) {
        this.robot = robot;
        this.opMode = opMode;
        this.robotName = name;
        try {
            // Create Datalogger
            Dl = new DataLogger(robotName + " OdometryLog" + System.currentTimeMillis() + ".csv");
            Dl.addHeaderLine("Systemtime", "local time", "rotatation", "x", "y", "deltaX", "deltaY");

            // Update the log file
        } catch (IOException e){

        }
    }
    @Override
    public void run() {
        //on start, keep running unless stop is requested
        while (!opMode.isStopRequested()) {
            //call the robot updateOdometry class
            double[] newData = robot.updateOdometry();
            try {
                Dl.addDataLine(System.currentTimeMillis(), localTime.seconds(), newData[0], newData[1], newData[2], newData[3], newData[4]);
            } catch (IOException e) {
                e.printStackTrace();
            }
            try {
                //pause for x amount of ms to prevent CPU overload
                Thread.sleep(robot.odometryUpdateRate);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        Dl.close();
    }
}
