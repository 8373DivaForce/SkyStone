package org.firstinspires.ftc.teamcode.Hardware_Maps;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.getMaxAbs;

public class NewKissHardware extends RobotConstructor {
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
    public final DcMotor dcFrontLeft;
    public final DcMotor dcFrontRight;
    public final DcMotor dcBackLeft;
    public final DcMotor dcBackRight;

    private final static String name = "Kissbot";
    public NewKissHardware(LinearOpMode opMode, double rotation, FunctionLibrary.Point position) {
        this(opMode,rotation, position,true);
    }
    public NewKissHardware(LinearOpMode opMode) {
        this(opMode,0,new FunctionLibrary.Point(0,0), false);
    }
    public NewKissHardware(LinearOpMode opMode, double rotation, FunctionLibrary.Point position, boolean enableOdometry) {
        super(opMode, name, wheelDiameter, dKp, minMoveSpeed,rampingDistance, CameraForwardDisplacement, CameraLeftDisplacement, CameraVerticalDisplacement, Webcamname, VuforiaKey, odometryUpdateRate);
        setRotation(rotation);
        setPosition(position);

        HardwareMap hMap = opMode.hardwareMap;

        dcFrontLeft = hMap.dcMotor.get("frontLeft");
        dcFrontRight = hMap.dcMotor.get("frontRight");
        dcBackLeft = hMap.dcMotor.get("backLeft");
        dcBackRight = hMap.dcMotor.get("backRight");

        expansionHub = hMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        dcBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        dcFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        dcFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        dcFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticksPerRev = (wheelDiameter*Math.PI)/dcFrontRight.getMotorType().getTicksPerRev();
        inchesPerTickX = ticksPerRev * 0.99469216609;
        inchesPerTickY = ticksPerRev* 1.00054933016;
        if (enableOdometry) {
            initOdometry();
        }
    }
        @Override
    public void move(double y, double x, double rotation, double power) {
        y = -y;
        double pFrontLeft= y + x;
        double pFrontRight = y - x;
        double pBackLeft= y - x;
        double pBackRight = y + x;

        double scaler = FunctionLibrary.getMaxAbs(pFrontLeft,pFrontRight,pBackLeft,pBackRight);
        if (scaler > power) {
            scaler = power/scaler;
            pFrontLeft *= scaler;
            pFrontRight *= scaler;
            pBackLeft *= scaler;
            pBackRight *= scaler;
        }
        pFrontLeft -= rotation;
        pFrontRight += rotation;
        pBackLeft -= rotation;
        pBackRight += rotation;
        scaler = FunctionLibrary.getMaxAbs(pFrontLeft,pFrontRight,pBackLeft,pBackRight);
        if (scaler > power) {
            scaler = power / scaler;
            pFrontLeft *= scaler;
            pFrontRight *= scaler;
            pBackLeft *= scaler;
            pBackRight *= scaler;
        }
            Log.d("move", getMaxAbs(pFrontLeft,pFrontRight,pBackLeft,pBackRight) +"");
        dcFrontLeft.setPower(pFrontLeft);
        dcFrontRight.setPower(pFrontRight);
        dcBackLeft.setPower(pBackLeft);
        dcBackRight.setPower(pBackRight);
    }

    double lastFrontLeft = 0;
    double lastFrontRight = 0;
    double lastBackLeft = 0;
    double lastBackRight = 0;
    private volatile double gyroRotation = 0;
    private volatile double worldRotation = 0;
    private volatile double rotationOffset = 0;
    public volatile RevBulkData bulkData;
    @Override
    public double[] updateOdometry() {

        bulkData = expansionHub.getBulkInputData();
        double[] results = super.updateOdometry();
        if (runOdometry) {
            double currentFrontLeft = bulkData.getMotorCurrentPosition(dcFrontLeft);
            double currentFrontRight = bulkData.getMotorCurrentPosition(dcFrontRight);
            double currentBackLeft = bulkData.getMotorCurrentPosition(dcBackLeft);
            double currentBackRight = bulkData.getMotorCurrentPosition(dcBackRight);

            double offsetFrontLeft = currentFrontLeft-lastFrontLeft;
            double offsetFrontRight = currentFrontRight-lastFrontRight;
            double offsetBackLeft = currentBackLeft-lastBackLeft;
            double offsetBackRight = currentBackRight-lastBackRight;

            lastFrontLeft = currentFrontLeft;
            lastFrontRight = currentFrontRight;
            lastBackLeft = currentBackLeft;
            lastBackRight = currentBackRight;

            double rotation = results[0];
            double yOffset = ((offsetFrontLeft + offsetFrontRight + offsetBackLeft + offsetBackRight)/4)*inchesPerTickY;
            double xOffset = ((offsetFrontLeft - offsetFrontRight - offsetBackLeft + offsetBackRight)/4)*inchesPerTickX;
            //double deltaX = yOffset*Math.sin(rotation) + xOffset*Math.cos(rotation);
            //double deltaY = yOffset*Math.cos(rotation) - xOffset*Math.sin(rotation);
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
