package org.firstinspires.ftc.teamcode.Hardware_Maps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.RobotConstructor;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class D1V4hardware extends RobotConstructor {
    //setup initial parameters that are provided to the parent robotconstructor class
    private static final String VuforiaKey = "AUJrAPb/////AAAAGV6Dp0zFW0tbif2eZk4u4LsrIQNxlQdiTbA2UJgYbEh7rb+s+Gg9soHReFwRRQz9xAiUcZi6d4jtD9+keLWR9xwcT+zJFSfdajjl89kWcf99HIxpWIMuNfAKhW83arD48Jnz/MTYxuBajilzcUxcPYQx24G/MeA6ZlyBhEauLXCKVrsdddL9kaEatPQx1MblEiH5wbdsMsXHz7w0B9CyEhQyZRLXb0zSbijn+JhHaHblBEk40x7gxkQYM1F+f+GfTrx5xR7ibvldNjRJ0obz1NJfuZugfW4R4vpV3C8Qebk7Jmy4YdL62Kb8W2Xk/S55jDhsdNW8rCPvVGJqjM5useObvRhomu0UT5EDH6hwOYxU";
    private static final String Webcamname = "Webcam 1";
    private static double wheelDiameter = 4.8;
    private static double dKp = 0.05;
    private static double minMoveSpeed = 0.1;
    public final static float CameraForwardDisplacement = (float)6.75;
    public final static float CameraLeftDisplacement = (float)1.5;
    public final static float CameraVerticalDisplacement = (float)5.25;
    private static float rampingDistance = 12;
    private static int odometryUpdateRate = 50;

    public final double inchesPerTickX;
    public final double inchesPerTickY;

    //initialize the variables for the hardware devices
    public final DcMotor dcFrontLeft;
    public final DcMotor dcFrontRight;
    public final DcMotor dcBackLeft;
    public final DcMotor dcBackRight;
    public final DcMotor dcUpDown1;
    public final DcMotor dcUpDown2;
    public final DcMotor dcInOut;
    public final DcMotor dcOpenClose;

    public final CRServo csRight;
    public final CRServo csLeft;

    public final Servo sHook;

    public final TouchSensor upperLimitSwitch;
    public final TouchSensor lowerLimitSwitch;


    //setup the constructor function
    public D1V4hardware(LinearOpMode opMode) {
        //provide the opMode given on creation as well as the variables defined above
        super(opMode, wheelDiameter, dKp, minMoveSpeed,rampingDistance, CameraForwardDisplacement, CameraLeftDisplacement, CameraVerticalDisplacement, Webcamname, VuforiaKey, odometryUpdateRate);
        //save the hardware map from the opMode
        HardwareMap hMap = opMode.hardwareMap;

        //set the variables to their corresponding hardware device
        dcFrontLeft = hMap.dcMotor.get("frontleft");
        dcFrontRight = hMap.dcMotor.get("frontright");
        dcBackLeft = hMap.dcMotor.get("backleft");
        dcBackRight = hMap.dcMotor.get("backright");
        dcInOut = hMap.dcMotor.get("inout");
        dcOpenClose = hMap.dcMotor.get("openclose");
        dcUpDown1 = hMap.dcMotor.get("updown1");
        dcUpDown2 = hMap.dcMotor.get("updown2");
        csRight = hMap.crservo.get("sright");
        csLeft = hMap.crservo.get("sleft");
        sHook = hMap.servo.get("shook");

        upperLimitSwitch = hMap.touchSensor.get("upperlimit");
        lowerLimitSwitch = hMap.touchSensor.get("downlimit");

        //setup the directions the devices need to operate in
        dcFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        dcBackLeft.setDirection(DcMotor.Direction.REVERSE);
        dcUpDown1.setDirection(DcMotor.Direction.REVERSE);
        dcUpDown2.setDirection(DcMotor.Direction.REVERSE);

        //make sure none of the devices are running
        dcFrontLeft.setPower(0);
        dcFrontRight.setPower(0);
        dcBackLeft.setPower(0);
        dcBackRight.setPower(0);
        dcOpenClose.setPower(0);
        dcUpDown1.setPower(0);
        dcUpDown2.setPower(0);
        dcInOut.setPower(0);

        csRight.setPower(0);
        csLeft.setPower(0);

        sHook.setPosition(0);

        //Reset the encoders on every motor
        dcFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcUpDown1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcUpDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcOpenClose.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set them to run without the encoders by default
        dcFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcUpDown1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcUpDown2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcInOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcOpenClose.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //initialize a variable useful in the odometry function
        inchesPerTickX = (1/dcFrontLeft.getMotorType().getTicksPerRev())*getWheelCircumfrance();
        inchesPerTickY = inchesPerTickX*1.08333333333;
    }
    //intialize the last encoder positions of the drive motors
    private double lastFrontLeftPos = 0;
    private double lastFrontRightPos = 0;
    private double lastBackLeftPos = 0;
    private double lastBackRightPos = 0;
    //a boolean that allows you to disable the position tracking
    boolean useOdometry = true;
    //overide the odometry function to make it robot specific
    @Override
    public void updateOdometry() {
        //calls that parent classes version of this function to update rotation
        super.updateOdometry();

        //check if odometry is enabled
        if (useOdometry) {
            //find the offset per wheel
            double frontLeftOffset = (dcFrontLeft.getCurrentPosition() - lastFrontLeftPos);
            double frontRightOffset = (dcFrontRight.getCurrentPosition() - lastFrontRightPos);
            double backLeftOffset = (dcBackLeft.getCurrentPosition() - lastBackLeftPos);
            double backRightOffset = (dcBackRight.getCurrentPosition() - lastBackRightPos);

            //set the last positions to the current ones for the next iteration
            lastFrontLeftPos = dcFrontLeft.getCurrentPosition();
            lastFrontRightPos = dcFrontRight.getCurrentPosition();
            lastBackLeftPos = dcBackLeft.getCurrentPosition();
            lastBackRightPos = dcBackRight.getCurrentPosition();

            //find the x and y offsets using inverse kinematics
            double yOffset = ((frontLeftOffset + frontRightOffset + backLeftOffset + backRightOffset)/4)*inchesPerTickY;
            double xOffset = (-(-frontLeftOffset + frontRightOffset + backLeftOffset - backRightOffset)/4)*inchesPerTickX;

            //find the hypotenuse to run trigonometry
            double hypot = sqrt(pow(xOffset, 2) + pow(yOffset, 2));

            //find the angle in which we moved
            double angle = Math.toDegrees(atan2(yOffset, xOffset));

            //adjust that angle by the current rotation to find the global movement
            double adjustedAngle = angle - getWorldRotation();

            //find the global x and y offsets using the hypot and translated angle
            double deltaX = hypot * cos(Math.toRadians(adjustedAngle));
            double deltaY = hypot * sin(Math.toRadians(adjustedAngle));

            //add the offsets to the global position
            addDeviation(new FunctionLibrary.Point(deltaX, deltaY));
        }
    }

    //overide the movement class
    @Override
    public void move(double x, double y, double rotation, double power) {
        //if power is zero, set the motors to brake
        if (power == 0) {
            for(DcMotor motors : getDriveMotors()) {
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            for (DcMotor motors : getDriveMotors()) {
                if (motors.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE)
                    motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        //invert the x input as it's flipped
        x = -x;
        //define initial kinimatics based off of mecanum drive
        double pFrontLeft= x + y;
        double pFrontRight = x - y;
        double pBackLeft= x - y;
        double pBackRight = x + y;

        //find the motor with the highest power level
        double max = max(max(abs(pFrontRight),abs(pFrontLeft)),max(abs(pBackRight), abs(pBackRight)));

        //reduce the total power if it's above 0.9 to ensure rotation will occur
        double drivePower = power;
        if (rotation > 0.1) drivePower = 0.9;

        //if that maximum power level is higher than the power given,
        //find the scaler value that will bring it down to that
        //and scale all of the motors using it
        if (max > drivePower) {
            double scaler = drivePower/max;
            pFrontLeft = pFrontLeft * scaler;
            pFrontRight = pFrontRight * scaler;
            pBackLeft = pBackLeft * scaler;
            pBackRight = pBackRight * scaler;
        }

        //add the rotation powers to the motors

        pFrontLeft = pFrontLeft + rotation;
        pFrontRight = pFrontRight - rotation;
        pBackLeft = pBackLeft + rotation;
        pBackRight = pBackRight - rotation;
        //find the motor with maximum power again
        max = max(max(abs(pFrontRight),abs(pFrontLeft)),max(abs(pBackRight), abs(pBackRight)));

        //if that maximum power level is higher than 1,
        //find the scaler value that will bring it down to that
        //and scale all of the motors using it
        if (max > 1) {
            double scaler = power/max;
            pFrontLeft = pFrontLeft * scaler;
            pFrontRight = pFrontRight * scaler;
            pBackLeft = pBackLeft * scaler;
            pBackRight = pBackRight * scaler;
        }

        //set all of the motors to those powers
        dcFrontLeft.setPower(pFrontLeft);
        dcFrontRight.setPower(pFrontRight);
        dcBackLeft.setPower(pBackLeft);
        dcBackRight.setPower(pBackRight);

        //check if the motors are set to run with encoders
        //if so, use kinimatics to find the encoder ticks they need to move
        //in order to reach the given position
        if (dcFrontLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {

            dcFrontLeft.setTargetPosition((int)(x+y));
            dcFrontRight.setTargetPosition((int)(x-y));
            dcBackLeft.setTargetPosition((int)(x-y));
            dcBackRight.setTargetPosition((int)(x+y));
        }
    }
    //override the getDriveMotors function in order to provide
    //the drivemotors to the autoFunctions class
    @Override
    public DcMotor[] getDriveMotors() {
        DcMotor[] motors = new DcMotor[4];
        motors[0] = dcFrontLeft;
        motors[1] = dcFrontRight;
        motors[2] = dcBackLeft;
        motors[3] = dcBackRight;
        return motors;
    }

    public void disableOdometry() {
        useOdometry = false;
    }
}
