package org.firstinspires.ftc.teamcode.Hardware_Maps;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Functions.RobotConstructor;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.getMaxAbs;

public class BeastBoyHardware extends RobotConstructor {
    //setup initial variables used to initialize the parent class
    private static final String VuforiaKey = "AUJrAPb/////AAAAGV6Dp0zFW0tbif2eZk4u4LsrIQNxlQdiTbA2UJgYbEh7rb+s+Gg9soHReFwRRQz9xAiUcZi6d4jtD9+keLWR9xwcT+zJFSfdajjl89kWcf99HIxpWIMuNfAKhW83arD48Jnz/MTYxuBajilzcUxcPYQx24G/MeA6ZlyBhEauLXCKVrsdddL9kaEatPQx1MblEiH5wbdsMsXHz7w0B9CyEhQyZRLXb0zSbijn+JhHaHblBEk40x7gxkQYM1F+f+GfTrx5xR7ibvldNjRJ0obz1NJfuZugfW4R4vpV3C8Qebk7Jmy4YdL62Kb8W2Xk/S55jDhsdNW8rCPvVGJqjM5useObvRhomu0UT5EDH6hwOYxU";
    private static final String Webcamname = "Front Webcam";
    private static double wheelDiameter = 4;
    private static double dKp = 0.05;
    private static double minMoveSpeed = 0.1;
    public static final float CameraForwardDisplacement = 0;
    public static final float CameraLeftDisplacement = 0;
    public static final float CameraVerticalDisplacement = 0;
    private static float rampingDistance = 12;
    private static int odometryUpdateRate = 9;

    public final double inchesPerTickX;
    public final double inchesPerTickY;

    public final ExpansionHubEx expansionHub;
    boolean runOdometry = true;
    //initialize the variables for the hardware devices
    public final DcMotor dcFrontLeft, dcFrontRight, dcBackLeft, dcBackRight;

    public final DcMotor dcIntakeLeft, dcIntakeRight, dcLift;

    public final Servo gate, gTopLeft, gTopRight, gBottomLeft, gBottomRight;
    public final Servo foundLeft, foundRight;

    public final LynxI2cColorRangeSensor blockRange;

    ExpansionHubEx frontExpansionHub;

    private final static String name = "Beast Boy";
    //setup initializers
    public BeastBoyHardware(LinearOpMode opMode, double rotation, FunctionLibrary.Point position) {
        this(opMode,rotation, position,true);
    }
    public BeastBoyHardware(LinearOpMode opMode) {
        this(opMode,0,new FunctionLibrary.Point(0,0), false);
    }
    //main initializer
    public BeastBoyHardware(LinearOpMode opMode, double rotation, FunctionLibrary.Point position, boolean enableOdometry) {
        super(opMode, name, wheelDiameter, dKp, minMoveSpeed,rampingDistance, CameraForwardDisplacement, CameraLeftDisplacement, CameraVerticalDisplacement, Webcamname, VuforiaKey, odometryUpdateRate);
        //set position and rotation
        setRotation(rotation);
        setPosition(position);

        HardwareMap hMap = opMode.hardwareMap;
        //set all the hardware devices to their corresponding variables
        blockRange = hMap.get(LynxI2cColorRangeSensor.class, "blockRange");
        dcFrontLeft = hMap.dcMotor.get("LMF");
        dcFrontRight = hMap.dcMotor.get("RMF");
        dcBackLeft = hMap.dcMotor.get("LMB");
        dcBackRight = hMap.dcMotor.get("RMB");

        dcIntakeLeft = hMap.dcMotor.get("IL");
        dcIntakeRight = hMap.dcMotor.get("IR");
        dcLift = hMap.dcMotor.get("lift");

        gate = hMap.servo.get("gate");
        gTopLeft = hMap.servo.get("topleft");
        gTopRight = hMap.servo.get("topright");
        gBottomLeft = hMap.servo.get("bottomleft");
        gBottomRight = hMap.servo.get("bottomright");

        foundLeft = hMap.servo.get("LFD");
        foundRight = hMap.servo.get("RFD");
        expansionHub = hMap.get(ExpansionHubEx.class, "Back Hub");
        frontExpansionHub = hMap.get(ExpansionHubEx.class, "Front Hub");

        //reverse the devices that need it
        foundLeft.setDirection(Servo.Direction.REVERSE);
        gTopLeft.setDirection(Servo.Direction.REVERSE);
        gBottomRight.setDirection(Servo.Direction.REVERSE);

        dcIntakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        dcBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        dcFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //stop and reset the encoders on the motors
        dcFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dcIntakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcIntakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set the motors to run without encoder for later use
        dcFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dcIntakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcIntakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize the servo positions to ensure they don't get in the way
        gate.setPosition(0);
        gTopLeft.setPosition(1);
        gTopRight.setPosition(1);
        gBottomLeft.setPosition(1);
        gBottomRight.setPosition(1);

        foundRight.setPosition(0);
        foundRight.setPosition(0);

        //setup calibrations for odometry
        double ticksPerRev = (wheelDiameter*Math.PI)/dcFrontRight.getMotorType().getTicksPerRev();
        inchesPerTickX = ticksPerRev * 0.91493570722;
        inchesPerTickY = ticksPerRev* 0.98831985624;
        if (enableOdometry) {
            initOdometry();
        }
    }
    //movement function for moving the robot based on the variables x, y, rotation, and power
    @Override
    public void move(double y, double x, double rotation, double power) {
        x = -x;
        //setup left and right movement
        double pFrontLeft= y + x;
        double pFrontRight = y - x;
        double pBackLeft= y - x;
        double pBackRight = y + x;
        //find the maximum magnitude
        double scaler = FunctionLibrary.getMaxAbs(pFrontLeft,pFrontRight,pBackLeft,pBackRight);
        //if the maximum is greater than the power, then scale the values
        if (scaler > power) {
            scaler = power/scaler;
            pFrontLeft *= scaler;
            pFrontRight *= scaler;
            pBackLeft *= scaler;
            pBackRight *= scaler;
        }
        //add rotation
        pFrontLeft -= rotation;
        pFrontRight += rotation;
        pBackLeft -= rotation;
        pBackRight += rotation;
        //find the maximum magnitude again
        scaler = FunctionLibrary.getMaxAbs(pFrontLeft,pFrontRight,pBackLeft,pBackRight);
        //scale them all once again
        if (scaler > power) {
            scaler = power / scaler;
            pFrontLeft *= scaler;
            pFrontRight *= scaler;
            pBackLeft *= scaler;
            pBackRight *= scaler;
        }
        //Apply the calculated powers to the motors
        dcFrontLeft.setPower(pFrontLeft);
        dcFrontRight.setPower(pFrontRight);
        dcBackLeft.setPower(pBackLeft);
        dcBackRight.setPower(pBackRight);
    }

    //setup variables for odometry
    double lastFrontLeft = 0;
    double lastFrontRight = 0;
    double lastBackLeft = 0;
    double lastBackRight = 0;
    public volatile RevBulkData bulkData;
    @Override
    public double[] updateOdometry() {

        bulkData = expansionHub.getBulkInputData();
        double[] results = super.updateOdometry();
        if (runOdometry) {
            //find the current position of each of the drive motors
            double currentFrontLeft = -bulkData.getMotorCurrentPosition(dcBackLeft);
            double currentFrontRight = -bulkData.getMotorCurrentPosition(dcBackRight);
            double currentBackLeft = bulkData.getMotorCurrentPosition(dcFrontRight);
            double currentBackRight = bulkData.getMotorCurrentPosition(dcFrontLeft);

            //find how much the motors have moved since last time
            double offsetFrontLeft = currentFrontLeft-lastFrontLeft;
            double offsetFrontRight = currentFrontRight-lastFrontRight;
            double offsetBackLeft = currentBackLeft-lastBackLeft;
            double offsetBackRight = currentBackRight-lastBackRight;

            //set the last positions to the current ones for next iteration
            lastFrontLeft = currentFrontLeft;
            lastFrontRight = currentFrontRight;
            lastBackLeft = currentBackLeft;
            lastBackRight = currentBackRight;

            //set rotation to the value returned by our super.updateOdometry() call
            double rotation = results[0];
            //find the x and y offsets using our calibrations and inverse kinimatics
            double yOffset = ((offsetFrontLeft + offsetFrontRight + offsetBackLeft + offsetBackRight)/4)*inchesPerTickY;
            double xOffset = ((offsetFrontLeft - offsetFrontRight - offsetBackLeft + offsetBackRight)/4)*inchesPerTickX;
            //find the hypot of a triangle made up of our x and y offsets
            double hypot = sqrt(pow(xOffset, 2) + pow(yOffset, 2));

            //find the angle in which we moved
            double angle = Math.toDegrees(atan2(yOffset, xOffset));

            //adjust that angle by the current rotation to find the global movement
            double adjustedAngle = angle - rotation;

            //find the global x and y offsets using the hypot and translated angle
            double deltaX = hypot * cos(Math.toRadians(adjustedAngle));
            double deltaY = hypot * sin(Math.toRadians(adjustedAngle));

            addDeviation(new FunctionLibrary.Point(deltaX,deltaY));
            return new double[] {
                    results[0],
                    getX(),
                    getY(),
                    deltaX,
                    deltaY
            };
        }
        return results;
    }
}
