package org.firstinspires.ftc.teamcode.Functions;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

import static java.lang.Math.abs;
import static java.lang.Math.log;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

//a class dedicated to various class or functions needed for programming
public class FunctionLibrary {
    //a class that can be used to control any given motor through encoder ticks
    public static class motorMovement {
        //define the motor
        private double timerOffset = 0;
        private double startingTimerOffset = 0;
        private final DcMotor[] motors;
        private int nState = 0;
        ElapsedTime timer = new ElapsedTime();
        private final int minTicksPerCycle;
        int lastState[];
        int[] startingPosition;
        private TouchSensor forwardLimit = null;
        private TouchSensor backwardLimit = null;

        //define the inputs needed to create a new instance of the class
        public motorMovement(int minTicksPerCycle, DcMotor... motors) {
            //take motor input and assign it to the stored motor variable
            this.motors = motors;
            lastState = new int[motors.length];
            this.minTicksPerCycle = minTicksPerCycle/200;
            startingPosition = new int[motors.length];
        }
        public void limits(TouchSensor forwardLimit, TouchSensor backwardLimit) {
            this.forwardLimit = forwardLimit;
            this.backwardLimit = backwardLimit;
        }

        //allows the motor to be moved using encoder ticks
        // 0: Starting, reset encoders
        // 1: Check if encoders have been reset. If they aren't, reset them again. If they are, move on to the next state
        // 2: encoders have been reset, start the movement
        // -1: movement is done
        // -2: program timeout
        // -3: limit switches
        public int move_using_encoder(int ticks, double power, double timeout, int maxError, boolean stopIfNotMoving) {
            int nReturn = 0;
            //find the time relative to when this function started getting called
            double relativeTime = timer.milliseconds()-startingTimerOffset;
            //find the amount of time since the last time we checked how fast the motors were moving
            double currentIterationTime = timer.milliseconds()-timerOffset;
            switch (nState) {
                case 0:
                    //reset the encoders
                    for (int i = 0; i < motors.length; i++) {
                        startingPosition[i] = motors[i].getCurrentPosition();
                        lastState[i] = 1000000000;
                    }
                    //reset the timer for the timeout
                    timer.reset();
                    //set the return to go to the next state
                    nReturn = 1;
                    nState = 1;
                    break;
                case 1:
                    //check if encoders have been reset properly, if not reset them again
                    boolean allAreReset = true;
                    for (int i = 0; i < motors.length; i++) {
                        if (motors[i].getCurrentPosition() - startingPosition[i] > 50) {
                            startingPosition[i] = motors[i].getCurrentPosition();
                            lastState[i] = 100000000;
                            allAreReset = false;
                        }
                    }
                    if (allAreReset) {
                        nReturn = 2;
                        timerOffset = timer.milliseconds();
                        startingTimerOffset = timer.milliseconds();
                        nState = 2;
                    } else {
                        nReturn = 1;
                        nState = 1;
                    }
                    break;
                case 2:
                    boolean done = false;
                    //check if the program has ran past the timeout, if so, stop the program and return timedout
                    if (timeout * 1000 < timer.milliseconds()) {
                        nState = 0;
                        for (DcMotor motor : motors) {
                            motor.setPower(0);
                            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                        return -2;
                    }
                    for (int i = 0; i < motors.length; i++) {
                        DcMotor motor = motors[i];
                        //set the motors power and position
                        motor.setPower(power);
                        motor.setTargetPosition(ticks + startingPosition[i]);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //find the current position relative to where it was when this function started being called
                        int currentPosition = motor.getCurrentPosition() - startingPosition[i];
                        //check if the motor has reached it's target
                        if (Math.abs(currentPosition - ticks) < maxError) {
                            //stop the motor
                            //tell the autonomous that the movement is done
                            nReturn = -1;
                            nState = 0;
                            done = true;
                        } else if (Math.abs(currentPosition - lastState[i])/currentIterationTime < minTicksPerCycle &&
                                stopIfNotMoving && currentIterationTime > 50 && relativeTime > 400) {
                            //stop the motors
                            //tell the autonomous that the motors met resistance
                            nReturn = -2;
                            nState = 0;
                            done = true;
                        } else if (currentIterationTime > 50) {//if it's been more than 50 ms since the last check
                                                               //reset it so it'll wait another 50 ms
                            timerOffset = timer.milliseconds();
                            lastState[i] = currentPosition;
                        }
                    }
                    if ((forwardLimit != null && backwardLimit != null) &&
                        ((forwardLimit.isPressed() && (ticks) > 0) ||
                        (backwardLimit.isPressed() && (ticks) < 0))) {
                        nReturn = -3;
                        done = true;
                    }
                    //check if any of the if statements say the program is done
                    //if so, turn off all attached motors
                    if (done) {
                        for (DcMotor motor : motors) {
                            motor.setPower(0);
                        }
                    }

            }
            //return the current state the function is in
            return nReturn;
        }

        //like the function above, except it uses a tick position relative to where the motor was on startup
        public int move_encoder_to_position(int ticks, double power, double timeout, int maxError, boolean stopIfNotMoving) {
            int maxStartingPosition = 0;
            int minStartingPosition = 0;
            //iterate through each motor to find which one has the highest tick value
            for (int i = 0; i < motors.length; i++) {
                startingPosition[i] = motors[i].getCurrentPosition();
                maxStartingPosition = max(startingPosition[i], maxStartingPosition);
                minStartingPosition = min(startingPosition[i], minStartingPosition);
            }

            //use the above function to move the necessary amount of ticks and return what that function does
            //move the amount of ticks needed according to the motor that is closest
            //to the desired amount of ticks in order to avoid any damage caused by
            //discrepancies between the two motor readings.
            if (abs(minStartingPosition-ticks) < abs(maxStartingPosition-ticks)) {
                return move_using_encoder(ticks-minStartingPosition, power, timeout, maxError, stopIfNotMoving);
            } else {
                return move_using_encoder(ticks-maxStartingPosition, power, timeout, maxError, stopIfNotMoving);
            }
        }
    }
    //function for retrieving the heading from a given gyroscope with an offset
    public static double GetYaw(double dOffsetgyro, BNO055IMU imu){
        double dFixCurHeading;
        //grab the imu orientation and apply the given offset
        dFixCurHeading = -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle-dOffsetgyro);

        //wrap the angle to fit within -180 and 180 degrees
        while (dFixCurHeading>180) {
            dFixCurHeading = dFixCurHeading-360;
        }

        while (dFixCurHeading<-180) {
            dFixCurHeading = dFixCurHeading+360;
        }
        //return the values it found
        return dFixCurHeading;
    }

    //function for wrapping a given angle to be between -180 and 180 degrees
    public static double WrapAngleDegrees(double degrees) {
        // if degrees is positive then divide it by 360 and take the remainder
        // if it is negative then divide it by -360 and take the remainder
        //this wraps the rotation to between 0 and 360 or -360 and 0
        degrees = degrees > 0 ? degrees%360 : degrees%-360;

        //if degrees is greater than 180, subtract 360 to make it be between -180 and 180
        degrees = degrees > 180 ? degrees-360 : degrees;
        //if degrees is less than -180, add 360 to make it between -180 and 180
        degrees = degrees < -180 ? degrees+360 : degrees;

        //return the value
        return degrees;
    }
    //constructor class for a point containing an x and y position
    public static class Point {
        //define the coordinates x and y as doubles
        public double x;
        public double y;
        //take the x and y on creation of an instance of point
        //and map them to the corresponding variable
        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    //lineCircleintersection function for PurePursuit which I haven't implemented yet
    public static ArrayList<Point> lineCircleIntersection(Point center, double Radius, Point linePoint1, Point linePoint2) {

        //make sure that the distance slope between points isn't too small so we can calculate slope
        linePoint2.x = abs(linePoint1.x-linePoint2.x) < 0.005 ? linePoint2.x+0.006 : linePoint2.x;
        linePoint2.y = abs(linePoint1.y-linePoint2.y) < 0.005 ? linePoint2.y+0.006 : linePoint2.y;

        //sets the line points to be relative to the center point
        Point localPoint1 = new Point(linePoint1.x-center.x, linePoint1.y-center.y);
        Point localPoint2 = new Point(linePoint2.x-center.x, linePoint2.y-center.y);

        Log.d("Points", "point1: " + localPoint1.x + ", " + localPoint1.y + "; Point2: " + localPoint2.x + ", " + localPoint2.y);
        //define m in y=mx+b  for a linear line
        double m = (localPoint2.y-localPoint1.y)/(localPoint2.x-localPoint1.x);
        //define b in y=mx+B for a linear line
        double b = localPoint2.y-m*localPoint2.x;

        Log.d("Points", "m: " + m + ", b: " + b + ", r: " + Radius);
        //set Radius equal to r
        double r = Radius;

        //create an arraylist of points as there can be 0, 1, or 2 points
        ArrayList<Point> Points = new ArrayList<>();
        try {
            //define parts A, B, and C of an equation similar to the quadratic formula
            double partA = m*b;
            double partB = sqrt(pow(Radius,2) + (pow(m,2) * pow(Radius,2)) - pow(b,2));
            double partC = pow(m,2) + 1;

            Log.d("Points", "A: " + partA + ", B: " + partB + ", C: " + partC);
            //get the x of one point
            double x1 = (-partA + partB)/partC;
            //get the x of the other point
            double x2 = (partA + partB)/partC * -1;

            Log.d("Points", "X1: " + x1 + ", X2: " + x2);

            //transition local x coordinates back to global and calculate y using y = mx+b
            Point globalPoint1 = new Point(x1+center.x, (x1*m + b) + center.y);
            Point globalPoint2 = new Point(x2+center.x,(x2*m+b) + center.y);
            Log.d("Points", "Point1: " + globalPoint1.x + ", " + globalPoint1.y + "; Point2: " + globalPoint2.x + ", " + globalPoint2.y);

            double minX;
            double maxX;
            if (linePoint1.x < linePoint2.x) {
                minX = linePoint1.x;
                maxX = linePoint2.x;
            } else {
                minX = linePoint2.x;
                maxX = linePoint1.x;
            }

            Log.d("Points", "Min: " + minX + ", Max: " + maxX);
            //check if the point we found is on the line
            //if so, add it to the list of points found
            if (globalPoint1.x > minX && globalPoint1.x < maxX) {
                Points.add(globalPoint1);
            }
            //check if the other point we found is on the line
            //if so, add it to the list of points found
            if (globalPoint2.x > minX && globalPoint2.x < maxX) {
                Points.add(globalPoint2);
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
        //return the points that were found
        return Points;
    }
    public static double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
    public static double getMax(double ... values) {
        double maxVal = 0;
        for (int i = 0; i < values.length; i++) {
            if (maxVal < values[i]) maxVal = values[i];
        }
        return maxVal;
    }
    public static double getMaxAbs(double ... values) {
        double[] vals = new double[values.length];
        for (int i = 0; i < vals.length; i++) {
            vals[i] = abs(values[i]);
        }
        return getMax(vals);
    }
    public static double[] mergeDoubleArrays(double ... values) {
        return values;
    }
    public static double[] scaleDownValues(double max, double ... values) {
        max = abs(max);
        double maxVal = getMaxAbs(values);
        if (max < maxVal) {
            double scaler = max/maxVal;
            for (int i = 0; i < values.length; i++) {
                values[i] *= scaler;
            }
        }
        return values;
    }

}
