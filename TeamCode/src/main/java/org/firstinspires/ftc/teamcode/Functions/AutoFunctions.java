package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

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

    //this is the constant for error correction in PID.
    //This takes in the robotConstructor object
    public AutoFunctions(RobotConstructor robot) {
        //saves the robot
        this.robot = robot;
        //initialize the gyro
        try {
            // Create Datalogger
            Dl = new DataLogger("AutoLog.csv");
            Dl.addHeaderLine("Time", "rotation", "error", "LeftRight Target", "FrontBack Target", "Front Power", "Back Power", "Left Power","Right Power");

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
                //multiply the error by dKP value to get rotational correction
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
    public int rotPID(double dAngle, double dPower, int nMaxError, int nTimeout) {
        int nReturn = 0;
        int direction = (int)(Math.abs(dAngle)/dAngle);
        switch(nState) {
            case 0:
                timer.reset();
                set_drive_encoders(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                nState = 1;
                break;
            case 1:
                //normalize the angle to be within -360 and 360 degrees
                if (dAngle > 0) dAngle = dAngle%360;
                if (dAngle < 0) {
                    dAngle = dAngle%-360;
                    dAngle = 360+dAngle;
                }
                //take the current rotation
                double gyroAngle = robot.getWorldRotation();
                double rotation;
                if (gyroAngle < 0) {
                    rotation = gyroAngle+360;
                } else {
                    rotation = gyroAngle;
                }
                //compare current rotation to targetAngle to get how much the robot needs to turn
                double dRotation = (rotation-dAngle);


                //divide the rotational difference and divide by 360 to get power
                double dRotationPow = dRotation/360;

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
                    for (DcMotor motor : robot.getDriveMotors()) {
                        motor.setPower(0);
                    }
                    //return -1 to signify the program finished successfully.
                    nReturn = -1;
                }
                //check if the programmer has been going longer than it's supposed to
                //if it is, stop all motors, reset state, and return -2 to signify a timeout.
                if (timer.milliseconds() > nTimeout*1000) {
                    nState = 0;
                    for (DcMotor motor : robot.getDriveMotors()) {
                        motor.setPower(0);
                    }
                    nReturn = -2;
                }
        }

        return nReturn;
    }
    //takes a destination point, max power, and max error and navigates to that point using odometry data
    public int gotoPosition(FunctionLibrary.Point destination,double power, double error) {
        //find the x and y offset using odometry position and destination
        double xPos = destination.x-robot.getX();
        double yPos = destination.y-robot.getY();
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
        //translate that to the global movement vector
        double robotAngle = angle + robot.getWorldRotation();

        //find the x movement using the distance and cosine of the adjusted angle
        double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
        //find the y movement using the distance and sin of the adjusted angle
        double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));

        //feeds those values into the robot move function
        robot.move(adjustedY, adjustedX, 0, power);

        return 0;
    }
    //same function as before, but it tries to stay at the same heading throughout the movement
    public int gotoPosition(FunctionLibrary.Point destination,double power, double error, double targetAngle) {
        //finds the local x and y offset
        double xPos = destination.x-robot.getX();
        double yPos = destination.y-robot.getY();
        //finds the distance from the points
        double distance = Math.sqrt((xPos*xPos) + (yPos*yPos));
        //checks if that distance is less than the max error
        //if so, stop the drive motors and return -1
        if (distance < error) {
            robot.move(0,0,0,0);
            return -1;
        }
        //find the current angle
        double currentAngle = robot.getWorldRotation();
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

        return 0;
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
