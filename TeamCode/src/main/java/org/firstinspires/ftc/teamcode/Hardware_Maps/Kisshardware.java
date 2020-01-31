package org.firstinspires.ftc.teamcode.Hardware_Maps;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

public class Kisshardware extends RobotConstructor {
    //setup initial parameters that are provided to the parent robotconstructor class
    private static final String VuforiaKey = "AUJrAPb/////AAAAGV6Dp0zFW0tbif2eZk4u4LsrIQNxlQdiTbA2UJgYbEh7rb+s+Gg9soHReFwRRQz9xAiUcZi6d4jtD9+keLWR9xwcT+zJFSfdajjl89kWcf99HIxpWIMuNfAKhW83arD48Jnz/MTYxuBajilzcUxcPYQx24G/MeA6ZlyBhEauLXCKVrsdddL9kaEatPQx1MblEiH5wbdsMsXHz7w0B9CyEhQyZRLXb0zSbijn+JhHaHblBEk40x7gxkQYM1F+f+GfTrx5xR7ibvldNjRJ0obz1NJfuZugfW4R4vpV3C8Qebk7Jmy4YdL62Kb8W2Xk/S55jDhsdNW8rCPvVGJqjM5useObvRhomu0UT5EDH6hwOYxU";
    private static String Webcamname;
    private static double wheelDiameter = 4.8;
    private static double dKp = 0.05;
    private static double minMoveSpeed = 0.1;
    public static float CameraForwardDisplacement;
    public static float CameraLeftDisplacement;
    public static float CameraVerticalDisplacement;
    private static float rampingDistance = 12;
    private static int odometryUpdateRate = 6;

    public final double inchesPerTickX;
    public final double inchesPerTickY;

    //initialize the variables for the hardware devices
    public final DcMotor dcFrontLeft;
    public final DcMotor dcFrontRight;
    public final DcMotor dcBackLeft;
    public final DcMotor dcBackRight;


    public final Servo sLStoneHook;
    public final Servo sRStoneHook;
    private static float cameraChoice(String camera) {
        Webcamname = camera;
        if(camera == null) {
            Webcamname ="left";
        }
        CameraVerticalDisplacement = 0;
        CameraForwardDisplacement = 0;
        CameraLeftDisplacement = 0;
        return CameraForwardDisplacement;
    }

    private final static String name = "Kissbot";
    //setup the constructor function
    public Kisshardware(LinearOpMode opMode, double rotation, String camera) {
        //provide the opMode given on creation as well as the variables defined above
        super(opMode, name, wheelDiameter, dKp, minMoveSpeed,rampingDistance, cameraChoice(camera), CameraLeftDisplacement, CameraVerticalDisplacement, Webcamname, VuforiaKey, odometryUpdateRate);
        //save the hardware map from the opMode
        HardwareMap hMap = opMode.hardwareMap;

        //set the variables to their corresponding hardware device
        dcFrontLeft = hMap.dcMotor.get("frontleft");
        dcFrontRight = hMap.dcMotor.get("frontright");
        dcBackLeft = hMap.dcMotor.get("backleft");
        dcBackRight = hMap.dcMotor.get("backright");

        sRStoneHook = hMap.servo.get("rightstone");
        sLStoneHook = hMap.servo.get("leftstone");
        //setup the directions the devices need to operate in
        dcFrontRight.setDirection(DcMotor.Direction.REVERSE);
        dcBackRight.setDirection(DcMotor.Direction.REVERSE);
        sRStoneHook.setDirection(Servo.Direction.REVERSE);

        //make sure none of the devices are running
        dcFrontLeft.setPower(0);
        dcFrontRight.setPower(0);
        dcBackLeft.setPower(0);
        dcBackRight.setPower(0);

        sLStoneHook.setPosition(0);
        sRStoneHook.setPosition(0);

        //Reset the encoders on every motor
        dcFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set them to run without the encoders by default
        dcFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize a variable useful in the odometry function
        double tempInchPerTick = (1/dcFrontLeft.getMotorType().getTicksPerRev())*getWheelCircumfrance();
        // old inchesPerTickX = tempInchPerTick*-0.92307692307692307692307692307692;
        inchesPerTickX = tempInchPerTick*-1.11089265;

        // old inchesPerTickY = tempInchPerTick*1.1483253588516746411483253588517;
        inchesPerTickY = tempInchPerTick*-1.22675109;
        useOdometry = false;
        setRotation(rotation);
        initOdometry();
    }
    public Kisshardware(LinearOpMode opMode, double x, double y, double rotation) {
        this(opMode, rotation, null);
        setPosition(x,y);
        useOdometry = true;
    }
    public Kisshardware(LinearOpMode opMode, double x, double y, double rotation, String camera) {
        this(opMode, rotation, camera);
        setPosition(x,y);
        useOdometry = true;
    }
    public Kisshardware(LinearOpMode opMode, FunctionLibrary.Point startingPos, double rotation) {
        this(opMode, startingPos.x, startingPos.y, rotation);
    }
    public Kisshardware(LinearOpMode opMode, FunctionLibrary.Point startingPos, double rotation, String camera) {
        this(opMode, startingPos.x, startingPos.y, rotation,camera);
    }
    //intialize the last encoder positions of the drive motors
    private double lastFrontLeftPos = 0;
    private double lastFrontRightPos = 0;
    private double lastBackLeftPos = 0;
    private double lastBackRightPos = 0;
    //a boolean that allows you to disable the position tracking
    boolean useOdometry;
    //overide the odometry function to make it robot specific
    @Override
    public double[] updateOdometry() {
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

            Log.d("Odometry Update", "yOffset: " + yOffset + " xOffset: " + xOffset + "FrontLeft: " + frontLeftOffset + "FrontRight: " + frontRightOffset + "BackLeft" + backLeftOffset + " BackRight: " + backRightOffset);
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
            return new double[] {
                    getWorldRotation(),
                    getX(),
                    getY(),
                    deltaX,
                    deltaY
            };
        }
        return new double[] {
                getWorldRotation(),
                getX(),
                getY(),
                0,
                0
        };
    }

    //overide the movement class
    @Override
    public void move(double x, double y, double rotation, double power) {
        //invert the x input as it's flipped\
        //define initial kinimatics based off of mecanum drive
        y=-y;
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

        pFrontLeft = pFrontLeft - rotation;
        pFrontRight = pFrontRight + rotation;
        pBackLeft = pBackLeft - rotation;
        pBackRight = pBackRight + rotation;
        //find the motor with maximum power again
        max = max(max(abs(pFrontRight),abs(pFrontLeft)),max(abs(pBackRight), abs(pBackRight)));

        //if that maximum power level is higher than 1,
        //find the scaler value that will bring it down to that
        //and scale all of the motors using it
        if (max > power) {
            double scaler = power/max;
            pFrontLeft = pFrontLeft * scaler;
            pFrontRight = pFrontRight * scaler;
            pBackLeft = pBackLeft * scaler;
            pBackRight = pBackRight * scaler;
        }
        if (power < 0 || (Math.abs(pFrontLeft) == 0 && Math.abs(pFrontRight) == 0 && pBackLeft == 0 && pBackRight == 0)) {
            dcFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dcBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            dcFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            dcFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            dcBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            dcBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    public void enableOdometry() {
        useOdometry = true;
    }
}
