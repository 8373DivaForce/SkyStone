package org.firstinspires.ftc.teamcode.Functions;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;
import java.util.ArrayList;

//class dedicated for functions used in autonomous
public class AutoFunctions {
    //set up the initial variables
    private int nState = 0;
    private BNO055IMU imu;
    private final ElapsedTime timer = new ElapsedTime(); //create a timer used for timeouts
    // this value tells the program how big your wheels are in inches
    private final RobotConstructor robot;
    private double dOffsetGyro = 0;
    private DataLogger Dl;
    private Odometry odometry;
    private double startingAngle = 0;
    private final ElapsedTime localTime = new ElapsedTime();
    //this is the constant for error correction in PID.
    //This takes in the robotConstructor object
    public AutoFunctions(RobotConstructor robot) {
        //saves the robot
        this.robot = robot;
        //initialize the gyro
        try {
            // Create Datalogger
            Dl = new DataLogger(robot.name + " autolog " + System.currentTimeMillis() + ".csv");
            Dl.addHeaderLine("System Time", "Local Time", "Rotation", "X", "Y","Target Rotation", "TargetX", "TargetY");

            // Update the log file
        } catch (IOException e){

        }
    }
    //Stops the datalogger at the end of the program.
    public void close() {
        Dl.close();
    }

    //move robot at a certain angle and certain distance using drive encoders
    //The power should be greater than 0 and less than or equal to 1.
    //states:
    // 0: Starting, reset encoders
    // 1: Check if encoders have been reset. If they aren't, reset them again. If they are, move on to the next state
    // 2: encoders have been reset, start the movement
    // -1: movement is done
    // -2: program timeout
    public int MoveRotHolPID(double distance, double dAngle, double power, double timeout, double targetAngle) {
        int nReturn = 0; //tells autonomous what state the program is in and is used to go to the next state
        //go to the current state
        switch(nState) {
            case 0:
                set_drive_encoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset the encode
                timer.reset(); //reset the timer for the timeout
                nState = 1;
                nReturn = 1;
                break;
            case 1:
                //default return value for this state
                nState = 1;
                nReturn = 1;
                //overide nReturn value to 2 if the drive motor encoders are within the margin of error
                if (check_if_encoders_reset(100)) {
                    nReturn = 2;
                    nState = 2;
                }
                //if that doesn't work, reset the encoders again and repeat this step using the default nReturn value
                else set_drive_encoders(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 2:
                if ((int)timer.milliseconds() > timeout*1000) {
                    for (DcMotor motor : robot.getDriveMotors()) {
                        motor.setPower(0);
                    }
                    nState = 0;
                    return -2;
                }
                int ticks = (int)(distance/robot.getWheelCircumfrance() * robot.getDriveMotors()[0].getMotorType().getTicksPerRev());
                //calculates the amount of ticks the Left and Right motors need to move
                int nLeftRightMov = -(int)(Math.cos(Math.toRadians(dAngle))*ticks);
                //calculates the amount of ticks the Front and Back motors need to move
                int nFrontBackMov = (int)(Math.sin(Math.toRadians(dAngle))*ticks);

                // Takes the correction constant (dKP) and multiplies it by the deviation from the targetAngle to get the error.
                double dHeading = robot.getWorldRotation();
                if (dHeading < 0) dHeading += 360;

                if (targetAngle > 0) targetAngle = targetAngle%360;
                if (targetAngle < 0) {
                    targetAngle = targetAngle%-360;
                    targetAngle += 360;
                }
                //get the total rotational error
                double dError = FunctionLibrary.WrapAngleDegrees(dHeading - targetAngle);
                //multiply the error by getWheelCircumfrance()dKP value to get rotational correction
                dError = dError * robot.getdKp();
                //feed the move function in the robotConstructor the x, y, and z movement
                robot.move(-nFrontBackMov,nLeftRightMov, dError,power);

                set_drive_encoders(DcMotor.RunMode.RUN_TO_POSITION);
                if (check_drive_encoders(100)) {
                    //stop all of the motors
                    for (DcMotor motor : robot.getDriveMotors())  {
                        motor.setPower(0);
                    }
                    //tell the autonomous that this function is done
                    nState = 0;
                    nReturn = -1;
                }
                try {
                    // Create Datalogger
                    Dl.addDataLine(timer.milliseconds(),dHeading, dError, nLeftRightMov, nFrontBackMov);
                    // Update the log file
                } catch (IOException e){

                }
                break;
            default:
                // if an unknown value gets passed in, just return program timeout
                nReturn = 4;
                nState = 0;

        }
        return nReturn;
    }

    //states:
    // -2: timeout
    //-1: done
    //0: reset timer
    //1: in progress
    public int rotPID(double dAngle, double dPower, double nMaxError, int nTimeout) {
        int nReturn = 0;
        Log.d("ROTPID", nState + "");
        int direction = (int)(Math.abs(dAngle)/dAngle);
        switch(nState) {
            case 0:
                timer.reset();
                nState = 1;
                break;
            case 1:
                //normalize the angle to be within -360 and 360 degrees
                if (dAngle > 0) dAngle = dAngle%360;
                if (dAngle < 0) {
                    dAngle = dAngle%-360;
                }
                if (dAngle > 180) dAngle -= 360;
                if (dAngle < -180) dAngle += 360;

                double dAngle360 = dAngle;
                if (dAngle < 0) dAngle360 += 360;
                //take the current rotation
                double rotation = robot.getWorldRotation();
                double rotation360 = rotation;
                if (rotation < 0) rotation360 += 360;
                //compare current rotation to targetAngle to get how much the robot needs to turn
                double dRotation = (dAngle-rotation);
                double dRotation360 = (dAngle360-rotation360);

                if (Math.abs(dRotation360) < Math.abs(dRotation)) dRotation = dRotation360;

                //divide the rotational difference and divide by 360 to get power
                double dRotationPow = dRotation/45;

                //If the rotation power is greated than desired power scale it down
                if (dRotationPow > dPower) dRotationPow *= dPower/Math.abs(dRotationPow);
                //check to make sure the power isn't lower than min turn speed
                if (Math.abs(dRotationPow) < robot.getminMoveSpeed()) dRotationPow *= robot.getminMoveSpeed()/Math.abs(dRotationPow);
                //Tell the passed through movement function to turn
                robot.move(0,0,dRotationPow,1);
                //set the state to 1 again for the next iteration and set the return to 1
                nState = 1;
                nReturn = 1;
                //chest if the difference between the current rotation and target rotation
                //are with the allowed error value.
                if (Math.abs(rotation-dAngle) <= nMaxError) {
                    //reset the state
                    nState = 0;
                    //turn of the drive motors
                    robot.move(0,0,0,0);
                    //return -1 to signify the program finished successfully.
                    nReturn = -1;
                }
                //check if the programmer has been going longer than it's supposed to
                //if it is, stop all motors, reset state, and return -2 to signify a timeout.
                if (timer.milliseconds() > nTimeout*1000) {
                    nState = 0;
                    robot.move(0,0,0,0);
                    nReturn = -2;
                }
        }

        return nReturn;
    }
    public int turnDegrees(double degrees, double power, double maxError, double timeout) {
        int nReturn = 0;
        switch (nState) {
            case 0:
                startingAngle = robot.getWorldRotation();
                timer.reset();
                nState++;
                break;
            case 1:
                double rotation = robot.getWorldRotation();
                double worldAngle = startingAngle+degrees;
                double difference = Math.abs(worldAngle-rotation);
                double rotation360 = rotation;
                if (rotation < 0) rotation360 += 360;
                //compare current rotation to targetAngle to get how much the robot needs to turn
                double dRotation = (degrees-rotation);
                double dAngle360 = dRotation;
                double dRotation360 = (dAngle360-rotation360);
                while (dRotation360 < 0) dRotation360 += 360;

                if (Math.abs(dRotation360) < Math.abs(dRotation)) dRotation = dRotation360;

                double dPower = difference/50;
                if (dPower > power) dPower = dPower;
                robot.move(0,0,dPower, power);
                if (difference < maxError) {
                    nReturn = -1;
                    robot.move(0,0,0,0);
                } else if (timeout*1000 < timer.milliseconds()) {
                    nReturn = -2;
                    robot.move(0,0,0,0);
                }


        }
        return nReturn;
    }
    //takes a destination point, max power, and max error and navigates to that point using odometry data
    public int gotoPosition(FunctionLibrary.Point destination,double power, double error) {
        //find the x and y offset using odometry position and destination

        double globalX = robot.getX();
        double globalY = robot.getY();
        double xPos = destination.x-globalX;
        double yPos = destination.y-globalY;
        //find the total distance to the point using the pentagram formula
        double distance = Math.sqrt((xPos*xPos) + (yPos*yPos));
        //check if that distance is less than the max error
        //if so, set the drive motors to 0 and return -1
        if (distance < error) {
            robot.move(0,0,0,0);
            return -1;
        }
        //find the local movement Vector angle
        double angle = Math.toDegrees(Math.atan2(yPos,xPos));

        double currentRotation = robot.getWorldRotation();
        //translate that to the global movement vector
        double robotAngle = angle + currentRotation;

        //find the x movement using the distance and cosine of the adjusted angle
        double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
        //find the y movement using the distance and sin of the adjusted angle
        double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));

        //feeds those values into the robot move function
        robot.move(adjustedY, adjustedX, 0, power);
        try {
            Dl.addDataLine(System.currentTimeMillis(), localTime.seconds(), currentRotation, globalX, globalY, 1000, destination.x, destination.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return 0;
    }
    //same function as before, but it tries to stay at the same heading throughout the movement
    public int gotoPosition(FunctionLibrary.Point destination,double power, double error, double targetAngle) {
        double globalX = robot.getX();
        double globalY = robot.getY();
        //finds the local x and y offset
        double xPos = destination.x-globalX;
        double yPos = destination.y-globalY;
        //finds the distance from the points
        double distance = Math.sqrt((xPos*xPos) + (yPos*yPos));
        //checks if that distance is less than the max error
        //if so, stop the drive motors and return -1
        if (distance < error) {
            robot.move(0,0,0,0);
            return -1;
        }

        double currentRotation = robot.getWorldRotation();
        //find the current angle
        double currentAngle = currentRotation;
        //find the movement vector angle
        double angle = Math.toDegrees(Math.atan2(yPos,xPos));
        //translate the local movement vector to a global vecctor
        double robotAngle = angle + currentAngle;

        //find the x movement using distance times the cosine of the angle calculated
        double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
        //find the y movement using distance time the sin of the angle calculated
        double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));

        //find the current angle on a 0 to 360 degree span
        double currentAngle360 = currentAngle;
        if (currentAngle < 0) currentAngle360 = currentAngle+360;

        double targetAngle360 = targetAngle;
        if (targetAngle < 0) targetAngle360 = targetAngle+360;

        double robotRotation = 0;

        //check to see which way is faster, moving left or right and tell it to move accordingly
        if (Math.abs(targetAngle-currentAngle) < Math.abs(targetAngle360-currentAngle360)) {
            robotRotation = targetAngle-currentAngle;
        } else {
            robotRotation = targetAngle360-currentAngle360;
        }
        //pass the x movement, y movement, rotation, and power to the move function in the constructor class
        robot.move(adjustedY, adjustedX, robotRotation/50, power);
        try {
            Dl.addDataLine(System.currentTimeMillis(), localTime.seconds(), currentRotation, globalX, globalY, targetAngle, destination.x, destination.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return 0;
    }public int gotoPosition(FunctionLibrary.Point destination,double power, double error, boolean follow, double degToPow) {
        double globalX = robot.getX();
        double globalY = robot.getY();
        //finds the local x and y offset
        double xPos = destination.x-globalX;
        double yPos = destination.y-globalY;
        //finds the distance from the points
        double distance = Math.sqrt((xPos*xPos) + (yPos*yPos));
        //checks if that distance is less than the max error
        //if so, stop the drive motors and return -1
        if (distance < error) {
            robot.move(0,0,0,0);
            return -1;
        }

        double currentRotation = robot.getWorldRotation();
        //find the current angle
        double currentAngle = currentRotation;
        //find the movement vector angle
        double angle = Math.toDegrees(Math.atan2(yPos,xPos));
        //translate the local movement vector to a global vecctor
        double robotAngle = angle + currentAngle;

        //find the x movement using distance times the cosine of the angle calculated
        double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
        //find the y movement using distance time the sin of the angle calculated
        double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));
        double targetAngle = (angle-90)*-1;
        //find the current angle on a 0 to 360 degree span
        double currentAngle360 = currentAngle;
        if (currentAngle < 0) currentAngle360 = currentAngle+360;

        double targetAngle360 = targetAngle;
        if (targetAngle < 0) targetAngle360 = targetAngle+360;

        double robotRotation = 0;

        //check to see which way is faster, moving left or right and tell it to move accordingly
        if (Math.abs(targetAngle-currentAngle) < Math.abs(targetAngle360-currentAngle360)) {
            robotRotation = targetAngle-currentAngle;
        } else {
            robotRotation = targetAngle360-currentAngle360;
        }
        //pass the x movement, y movement, rotation, and power to the move function in the constructor class
        robot.move(adjustedY, adjustedX, robotRotation/degToPow, power);
        try {
            Dl.addDataLine(System.currentTimeMillis(), localTime.seconds(), currentRotation, globalX, globalY, targetAngle, destination.x, destination.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return 0;
    }
    FunctionLibrary.Point startingPos = new FunctionLibrary.Point(0,0);
    FunctionLibrary.Point initDestination = new FunctionLibrary.Point(0,0);
    public int gotoPosition(FunctionLibrary.Point destination,double power, double error, double targetAngle, double rampDistance) {
        double globalX = robot.getX();
        double globalY = robot.getY();
        if (initDestination.x != destination.x && initDestination.y != destination.y) {
            initDestination = destination;
            startingPos = new FunctionLibrary.Point(globalX, globalY);
        }
        //finds the local x and y offset
        double xPos = destination.x-globalX;
        double yPos = destination.y-globalY;
        //finds the distance from the points
        double distance = Math.sqrt((xPos*xPos) + (yPos*yPos));
        double distanceFromStart = Math.sqrt(Math.pow(globalX-startingPos.x,2)+Math.pow(globalY-startingPos.y,2));
        //checks if that distance is less than the max error
        //if so, stop the drive motors and return -1
        if (distance < error) {
            robot.move(0,0,0,0);
            return -1;
        }

        double currentRotation = robot.getWorldRotation();
        //find the current angle
        double currentAngle = currentRotation;
        //find the movement vector angle
        double angle = Math.toDegrees(Math.atan2(yPos,xPos));
        //translate the local movement vector to a global vecctor
        double robotAngle = angle + currentAngle;

        distanceFromStart = distanceFromStart/rampDistance;
        distance = distance/rampDistance;
        if (distanceFromStart < distance) distance = distanceFromStart;
        if (distance < 0.2) distance = 0.2;
        //find the x movement using distance times the cosine of the angle calculated
        double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
        //find the y movement using distance time the sin of the angle calculated
        double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));

        //find the current angle on a 0 to 360 degree span
        double currentAngle360 = currentAngle;
        if (currentAngle < 0) currentAngle360 = currentAngle+360;

        double targetAngle360 = targetAngle;
        if (targetAngle < 0) targetAngle360 = targetAngle+360;

        double robotRotation = 0;

        //check to see which way is faster, moving left or right and tell it to move accordingly
        if (Math.abs(targetAngle-currentAngle) < Math.abs(targetAngle360-currentAngle360)) {
            robotRotation = targetAngle-currentAngle;
        } else {
            robotRotation = targetAngle360-currentAngle360;
        }

        //pass the x movement, y movement, rotation, and power to the move function in the constructor class
        robot.move(adjustedY, adjustedX, robotRotation/50, power);
        try {
            Dl.addDataLine(System.currentTimeMillis(), localTime.seconds(), currentRotation, globalX, globalY, targetAngle, destination.x, destination.y);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return 0;
    }

    FunctionLibrary.Point lastPoint = new FunctionLibrary.Point(0,0);
    int lastLine = 0;
    public int purePursuit(FunctionLibrary.Point[] points, double radius, double maxPower, double maxError, double optimalAngle, Telemetry telemetry) {
        ArrayList<FunctionLibrary.Point> trackPoints = new ArrayList<>();
        FunctionLibrary.Point robotPos = robot.getPosition();
        FunctionLibrary.Point followingPoint = robotPos;
        int currentLine = 0;
        for (int i = 1; i < points.length; i++) {
            trackPoints = FunctionLibrary.lineCircleIntersection(robot.getPosition(), radius, points[i-1], points[i]);
            telemetry.addData("Line " + i, trackPoints.size());
            if (trackPoints.size() == 2) {
                double distance1 = Math.sqrt(Math.pow(trackPoints.get(0).x - points[i].x,2)+Math.pow(trackPoints.get(0).y - points[i].y,2));
                double distance2 = Math.sqrt(Math.pow(trackPoints.get(1).x - points[i].x,2)+Math.pow(trackPoints.get(1).y - points[i].y,2));

                if (distance1 < distance2) {
                    followingPoint = trackPoints.get(0);
                    currentLine = i;
                } else {
                    currentLine = i;
                    followingPoint = trackPoints.get(1);
                }
            } else if (trackPoints.size() == 1) {
                followingPoint = trackPoints.get(0);
                currentLine = i;
            }
        }
        int pointLen = points.length-1;
        if (lastLine > currentLine) {
            followingPoint = lastPoint;
        } else {
            lastLine = currentLine;
            lastPoint = followingPoint;
        }
        double distance1 = Math.sqrt(Math.pow(robotPos.x - points[pointLen].x,2)+Math.pow(robotPos.y - points[pointLen].y,2));
        double distance2 = Math.sqrt(Math.pow(robotPos.x - followingPoint.x,2)+Math.pow(followingPoint.y - points[pointLen].y,2));
        telemetry.addData("Following point", followingPoint.x + ", " + followingPoint.y);
        telemetry.addData("curentLine: ", currentLine);


        if (distance1 < maxError) {
            robot.move(0, 0, 0, 0);
            nState = 0;
            lastLine = 0;
            lastPoint = new FunctionLibrary.Point(0,0);
            return -1;
        } else if (distance1+1 < distance2 && currentLine == pointLen) {
            gotoPosition(points[pointLen],maxPower,1, optimalAngle, 12);
        } else {
            gotoPosition(followingPoint,maxPower,1, optimalAngle);
        }
        return currentLine;
    }
    //takes in a run mode like DcMotor.RunMode.STOP_AND_RESET_ENCODERS and applies it to every drive motor
    private void set_drive_encoders(DcMotor.RunMode runmode) {
        for (DcMotor motor : robot.getDriveMotors()) {
            motor.setMode(runmode);
        }
    }
    //take the maximum margin of error and return true if all drive motor enocder positions are below it
    private boolean check_if_encoders_reset(int errorMargin) {
        boolean bReturn = true;
        for (DcMotor motor : robot.getDriveMotors()) {
            if (motor.getCurrentPosition() > errorMargin) {
                bReturn = false;
            }
        }
        return bReturn;
    }
    //used if user supplies two int values. Will take a target and see if if the encoders are there within the error of margin.
    private boolean check_drive_encoders(int errorMargin) {
        boolean bReturn = false;
        for (DcMotor motor : robot.getDriveMotors()) {
            if (Math.abs(motor.getTargetPosition()) > errorMargin && Math.abs(motor.getTargetPosition()-motor.getCurrentPosition()) < errorMargin) {
                bReturn = true;
            }
        }
        return bReturn;
    }


}
