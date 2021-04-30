package org.firstinspires.ftc.teamcode.Libraries.functions;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.Bases.task;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;

//holder of some basic autonomous tasks
public class baseTasks {

    //will run a set of motors or servos until a touch sensor is hit
    public static class runUntil implements task {
        private DcMotor[] motors = null;
        private CRServo[] CRServos = null;
        private final TouchSensor[] sensors;
        private final double[] powers;


        //initialize program with touch sensor, motors, and the power it should run them at
        public runUntil(DcMotor[] motors, double[] powers, TouchSensor[] sensors) {
            this.motors = motors;
            this.powers = powers;
            this.sensors = sensors;
        }
        //knock off function for dealing with a single motor
        public runUntil(DcMotor motor, double power, TouchSensor[] sensors) {
            this(new DcMotor[]{motor}, new double[]{power},sensors);
        }
        //knock off function for dealing with a single power
        public runUntil(DcMotor[] motors, double power, TouchSensor[] sensors) {
            this.motors = motors;
            powers = new double[motors.length];
            for (int i = 0; i < motors.length; i++) {
                powers[i] = power;
            }
            this.sensors = sensors;
        }
        //same thing as motor functions above except it deals with CRServos
        public runUntil(CRServo[] CRServos, double[] powers, TouchSensor[] sensors) {
            this.CRServos = CRServos;
            this.powers = powers;
            this.sensors = sensors;
        }
        public runUntil(CRServo servo, double power, TouchSensor[] sensors) {
            this(new CRServo[]{servo}, new double[]{power}, sensors);
        }
        public runUntil(CRServo[] CRServos, double power, TouchSensor[] sensors) {
            this.CRServos = CRServos;
            powers = new double[CRServos.length];
            for (int i = 0; i < CRServos.length; i++) {
                powers[i] = power;
            }
            this.sensors = sensors;
        }
        @Override
        public void init() {
            //if we have motors, set all of them to run without encoders
            if (motors != null) {
                for (DcMotor motor : motors) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }

        @Override
        public int loop(RobotConstructor robot) {
            //if any of the touch sensors are being pressed, we are done and can end the program
            for (TouchSensor sensor : sensors) {
                if (sensor.isPressed()) {
                    //shut down motors or servos before ending program
                    if (motors != null) {
                        for (DcMotor motor : motors) motor.setPower(0);
                    } else {
                        for (CRServo servo : CRServos) servo.setPower(0);
                    }
                    //tell the task handler that we are done
                    return -1;
                }
            }
            //if we have motors, set them to their corresponding power
            if (motors != null) {
                if (motors.length != powers.length) return -3; //if we don't have a power for evey motor, terminate the program
                for (int i = 1; i < motors.length; i++) {
                    motors[i].setPower(powers[i]);
                }
            } else if(CRServos != null) { //if we have servos, set the to their corresponding power
                if (CRServos.length != powers.length) return -3; //if we don't have a power for every CRServo, terminate the program
                for (int i = 1; i < CRServos.length; i++) {
                    CRServos[i].setPower(powers[i]);
                }
            } else return -2; //if we somehow don't have servos or motors, terminate the program
            return 1; //if all else goes well, return that we are still running
        }
    }
    //basic odometry based movement task
    public static class move implements task {
        //initial variables needed for program
        double startTime = 0;
        final FunctionLibrary.Point targetPos;
        Double targetAngle = null;
        final double power;
        final double maxError;
        final double timeOut;
        double rampDistance = 8;
        double minPower = 0.1;

        //initialization function for getting where it needs to go, it's angle, it's power, maxError, and how long it should take
        public move(FunctionLibrary.Point pos, double angle, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.targetAngle = angle;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        public move(FunctionLibrary.Point pos, double angle, double power, double maxError, double timeOut, double rampDistance) {
            this(pos,angle,power,maxError,timeOut);
            this.rampDistance = rampDistance;
        }
        public move(FunctionLibrary.Point pos, double angle, double power, double maxError, double timeOut, double rampDistance, double minPower) {
            this(pos,angle,power,maxError,timeOut, rampDistance);
            this.minPower = minPower;
        }
        //same as above except without a target angle to keep the robot at
        public move(FunctionLibrary.Point pos, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        //set the robot's start time on init so we can have the program terminate after a set period of time
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //get the robots current position
            FunctionLibrary.Point pos = robot.getPosition();
            //if this task has taken to long, stop the robot and terminate the program
            if (System.currentTimeMillis()-startTime >= timeOut) {
                robot.move(0,0,0,0);
                return -2;
            //if the robot has reached it's target destination, stop the robots movement and terminate the program
            } else if (Math.sqrt(Math.pow(pos.x-targetPos.x,2) + Math.pow(pos.y-targetPos.y,2)) <= maxError) {
                robot.move(0,0,0,0);
                return -1;
            }
            //get it's offset from where it wants to be
            FunctionLibrary.Point offset = new FunctionLibrary.Point(targetPos.x-pos.x,targetPos.y-pos.y);
            double distance = Math.sqrt((offset.x*offset.x) + (offset.y*offset.y));
            double currentRotation = robot.getWorldRotation();
            //find the current angle
            double currentAngle = currentRotation;
            //find the movement vector angle
            double angle = Math.toDegrees(Math.atan2(offset.y,offset.x));
            //translate the local movement vector to a global vecctor
            double robotAngle = angle + currentAngle;

            //find the x movement using distance times the cosine of the angle calculated
            double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
            double xDir = Math.abs(adjustedX)/adjustedX;
            adjustedX = Math.abs(adjustedX)/rampDistance < minPower ? minPower*xDir : Math.abs(adjustedX/rampDistance) > power ? power*xDir : adjustedX/rampDistance;
            //find the y movement using distance time the sin of the angle calculated
            double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));
            double yDir = Math.abs(adjustedY)/adjustedY;
            adjustedY = Math.abs(adjustedY)/rampDistance < minPower ? minPower*yDir : Math.abs(adjustedY/rampDistance) > power ? power*yDir : adjustedY/rampDistance;


            //if it has an angle to go to, calculate how it needs to rotate, otherwise skip it
            if (targetAngle != null) {
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
            } else {
                //if we don't have a target rotation, just give it it's x and y movement
                robot.move(adjustedY, adjustedX, 0,power);
            }
            //return that the program is still running
            return 1;
        }
    }
    //task for moving motors
    public static class motorMovement implements task{
        //initial variables for auto task
        private final DcMotor[] motors;
        private final int[] positions;
        private final double maxError;
        private final double maxPower;
        private final double timeOut;
        private double startTime = 0;

        //initialization for values to use during auto
        public motorMovement(DcMotor[] motors, int[] positions, double maxPower, double maxError, double timeOut) {
            this.maxPower = maxPower;
            this.maxError = maxError;
            this.timeOut = timeOut;
            this.motors = motors;
            this.positions = positions;
        }
        //same as above except it takes in 1 motor instead of an array of them
        public motorMovement(DcMotor motor, int position, double maxPower, double maxError, double timeOut) {
            this.motors = new DcMotor[]{motor};
            this.positions = new int[]{position};
            this.maxPower = maxPower;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        @Override
        public void init() {
            //set the start time for the timeout and set the motors to run using encoders
            startTime = System.currentTimeMillis();
            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        @Override
        public int loop(RobotConstructor robot) {
            double error = 0;
            //check if the program has ran for too long, if so, terminate it
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;
            //if the amount of motors doesn't match the number of positions, terminate the program
            if (motors.length != positions.length) return -3;
            //iterate through each motor
            for (int i = 0; i < motors.length; i++) {
                //add the motors error to the total
                Log.d("RUNNING", "loop: ");
                error += Math.abs(motors[i].getCurrentPosition()-positions[i]);
                //set motor to the power level
                motors[i].setPower(maxPower);
                //set their target position to the one given
                motors[i].setTargetPosition(positions[i]);
                //tell the motors to run to that position
                motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //average out the error or distance from the motor's target
            error /= motors.length;
            //if the average error is within exceptable bounds, terminate the program
            if (error <= maxError) {
                //stop the motors
                for (DcMotor motor : motors) {
                    motor.setPower(0);
                }
                return -1;
            }
            //return that the program is still running if it has gotten to this point
            return 1;
        }
    }
    //task for controlling servos
    public static class servoMovement implements task {
        //setup initial variables
        private final Servo[] servos;
        private final double[] positions;
        private final double time;
        //initialization function to get the servos, there positions, and the amount of time to run for
        public servoMovement(Servo[] servos, double[] positions, double time) {
            this.servos = servos;
            this.positions = positions;
            this.time = time;
        }
        //same as above except it takes in a single servo and position
        public servoMovement(Servo servo, double position, double time) {
            this.servos = new Servo[]{servo};
            this.positions = new double[]{position};
            this.time = time;
        }
        private double startTime = 0;
        //set the initial start time so we can stop after x seconds
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //if the amounts of servos doesn't match the amount of positions, terminate the program
            if (servos.length != positions.length) return -3;
            //iterate through all of the servos and set them to their corresponding position
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(positions[i]);
            }
            //if the set number of seconds has passed, terminate the program
            if (System.currentTimeMillis()-startTime >= time) {
                return -1;
            }
            //return that the program is still running
            return 1;
        }
    }
    public static class wait implements task {
        //setup initial variables
        private final double time;
        //takes in the number of milliseconds to wait for
        public wait(double time) {
            this.time = time;
        }
        private double startTime = 0;
        //set the initial start time so we can stop after x milliseconds
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //if the set number of seconds has passed, terminate the program
            if (System.currentTimeMillis()-startTime >= time) {
                return -1;
            }
            //return that the program is still running
            return 1;
        }
    }
    public static class rotate implements task {
        //initial variables needed for program
        double startTime = 0;
        final double targetAngle;
        final double power;
        final double maxError;
        final double timeOut;
        //initialization function for getting the angle it needs to turn to, max power, maxError, and timeout
        public rotate(double angle, double power, double maxError, double timeOut) {
            this.targetAngle = angle;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        //set the robot's start time on init so we can have the program terminate after a set period of time
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }
        double timeStarted = 0;
        @Override
        public int loop(RobotConstructor robot) {
            //get the robots current rotation
            double currentRotation = robot.getWorldRotation();
            //if this task has taken to long, stop the robot and terminate the program
            if (System.currentTimeMillis()-startTime >= timeOut) {
                robot.move(0,0,0,0);
                return -2;
                //if the robot has reached it's target rotation, set a timer to check stability of angle
            } else if (timeStarted == 0 && Math.abs(currentRotation-targetAngle) <= maxError) {
                timeStarted = System.currentTimeMillis();

                return 1;
                //if robot angle is stable, return that the task is done
            } else if (timeStarted != 0 && System.currentTimeMillis()-timeStarted >= 100 && Math.abs(currentRotation-targetAngle) <= maxError) {
                robot.move(0,0,0,0);
                return -1;
                //if the robot angle is not stable, reset the timer
            } else if (Math.abs(currentRotation-targetAngle) > maxError){
                timeStarted = 0;
            }
            //find the current angle
            double currentAngle = currentRotation;

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
            //pass rotation and power to the move function in the constructor class
            robot.move(0, 0, robotRotation/8, power);
            //return that the program is still running
            return 1;
        }
    }
    public static class setMotorPower implements task {
        //initial variables needed for program
        final double power;
        final DcMotor[] motors;
        //initialization function for getting the motors and the power to set them to
        public setMotorPower(double power, DcMotor ... motors) {
            this.power = power;
            this.motors = motors;
        }
        @Override
        public void init() {
            for (DcMotor motor : motors) {
                if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    motor.setPower(0);
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }
        @Override
        public int loop(RobotConstructor robot) {
            //give the motors the given power
            for (DcMotor motor : motors) {
                motor.setPower(power);
            }
            //return that the task is finished
            return -1;
        }
    }
}
