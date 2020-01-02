package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.GetYaw;
import static org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.WrapAngleDegrees;

//my class for constructing all the parameters that are robot specific
public class RobotConstructor {

    //setup class values
    private final String VuforiaKey;

    private final Odometry odometry;
    public Thread odometryThread;

    private final float CameraForwardDisplacement;
    private final float CameraLeftDisplacement;
    private final float CameraVerticalDisplacement;
    public HashMap<String, VectorF> VuMarkPositions = new HashMap<>();
    public HashMap<String, Orientation> VuMarkOrientations = new HashMap<>();
    public VuforiaTrackables SkystoneTrackables;
    public ArrayList<VuforiaTrackable> SkystoneVuMarks;

    public final WebcamName webcam;
    public VuforiaLocalizer vuforia;

    public HardwareMap hMap;
    public final BNO055IMU imu;

    public final int odometryUpdateRate;

    private final double wheelDiameter;
    private final double dKp;
    private final double minMoveSpeed;

    private final double wheelCircumfrance;
    private final double rampingDistance;

    //initialize all of the variables for odometry
    //they are volatile to ensure thread safety
    private volatile double x = 0;
    private volatile double y = 0;
    private volatile double worldRotation = 0;
    private volatile double rotationOffset = 0;
    private volatile double gyroRotation = 0;

    boolean useWebcam = true;

    //constructor function, takes the opMode and the robot specific parameters
    public RobotConstructor(LinearOpMode opMode, double wheelDiameter, double dKp, double minMoveSpeed,
                            double rampingDistance, float CameraForwardDisplacement,
                            float CameraLeftDisplacement, float CameraVerticalDisplacement,
                            String webcameName, String VuforiaKey, int odometryUpdateRate) {
        //copy the variables that were passed through to the ones stored in the class
        this.hMap = opMode.hardwareMap;
        this.odometryUpdateRate = odometryUpdateRate;
        this.CameraForwardDisplacement = CameraForwardDisplacement;
        this.CameraLeftDisplacement = CameraLeftDisplacement;
        this.CameraVerticalDisplacement = CameraVerticalDisplacement;
        this.VuforiaKey = VuforiaKey;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumfrance = this.wheelDiameter*Math.PI;
        this.dKp = dKp;
        this.minMoveSpeed = minMoveSpeed;
        this.rampingDistance = rampingDistance;

        //setup for the initial IMU parameters
        BNO055IMU.Parameters BNparameters = new BNO055IMU.Parameters();
        BNparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNparameters.loggingEnabled = true;
        BNparameters.loggingTag = "IMU";
        BNparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //retrieve the imu
        imu = hMap.get(BNO055IMU.class, "imu");
        //initialize the imu
        imu.initialize(BNparameters);

        //create new odometry object
        this.odometry = new Odometry(this,opMode);
        //create a new thread off of the odometry object
        this.odometryThread = new Thread(odometry);
        //start the thread

        //define the webcame
        webcam = hMap.get(WebcamName.class, webcameName);
    }
    //same class as the one above except without a webcam
    public RobotConstructor(LinearOpMode opMode, double wheelDiameter, double dKp, double minMoveSpeed,
                            double rampingDistance, float CameraForwardDisplacement,
                            float CameraLeftDisplacement, float CameraVerticalDisplacement,
                            String VuforiaKey, int odometryUpdateRate) {
        //copy the variables that were passed through to the ones in the class
        this.hMap = opMode.hardwareMap;
        this.odometryUpdateRate = odometryUpdateRate;
        this.CameraForwardDisplacement = CameraForwardDisplacement;
        this.CameraLeftDisplacement = CameraLeftDisplacement;
        this.CameraVerticalDisplacement = CameraVerticalDisplacement;
        useWebcam = false;
        this.VuforiaKey = VuforiaKey;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumfrance = wheelDiameter*Math.PI;
        this.dKp = dKp;
        this.minMoveSpeed = minMoveSpeed;
        this.rampingDistance = rampingDistance;

        //setup initial IMU parameters
        BNO055IMU.Parameters BNparameters = new BNO055IMU.Parameters();
        BNparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNparameters.loggingEnabled = true;
        BNparameters.loggingTag = "IMU";
        BNparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //define the IMU from the hardwaremap
        imu = hMap.get(BNO055IMU.class, "imu");
        //initialize the IMU
        imu.initialize(BNparameters);

        //create new odometry object from the constructor class
        this.odometry = new Odometry(this,opMode);
        //create new thread from the odometry object
        this.odometryThread = new Thread(odometry);
        //start the odometry thread

        //set the webcam to null for other functions
        webcam = null;
    }

    //empty move class that takes an x, y, rotation, and max power
    //this function gets overridden by the robot specific extension of this class
    public void move(double x, double y, double rotation, double power) {

    }

    public void initOdometry() {
        if (!odometryThread.isAlive())
            odometryThread.start();
    }

    //function for the odometry thread to run
    //if left be, it only updates rotation
    //when overriden, you call super.updateOdometry() to update the rotation and then add inverse kinimatics
    //to calculate the position
    public void updateOdometry() {
        gyroRotation = GetYaw(0,imu);

        double rotation = gyroRotation+rotationOffset;
        if (rotation > 360) rotation %= 360;
        if (rotation < -360) rotation %= -360;
        if (rotation > 180) rotation -= 360;
        if (rotation < -180) rotation += 360;
        worldRotation = rotation;
    }
    //initializes vuforia for the 2019-2020 skystone challenge
    public void initVuforia(HardwareMap hMap) {
        InitSkystoneVuforia initVuforia = new InitSkystoneVuforia(hMap,webcam,VuforiaKey,CameraForwardDisplacement,CameraLeftDisplacement,CameraVerticalDisplacement);
        InitSkystoneVuforia.vuforiaData data = initVuforia.getVuforia();

        vuforia = data.vuforia;
        SkystoneVuMarks = data.vuMarks;
        SkystoneTrackables = data.trackables;
    }
    //iterates through all of the trackables and saves them to the HashMaps
    //VuMarkOrientations and vuMarkPositions which can be accessed by the main program
    public void updateVuforia() {
        VuMarkOrientations = new HashMap<>();
        VuMarkPositions = new HashMap<>();
        for (VuforiaTrackable trackable : SkystoneVuMarks) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    VectorF translation = robotLocationTransform.getTranslation();
                    VuMarkPositions.put(trackable.getName(), translation);
                    // express the rotation of the robot in degrees.
                    Orientation rotation = Orientation.getOrientation(robotLocationTransform, EXTRINSIC, XYZ, DEGREES);
                    VuMarkOrientations.put(trackable.getName(), rotation);
                }
            }
        }
    }

    //return and setter functions for class variables
    public double getWheelCircumfrance() {
        return wheelCircumfrance;
    }
    public double getdKp() {
        return dKp;
    }
    public double getminMoveSpeed() {
        return minMoveSpeed;
    }
    public DcMotor[] getDriveMotors() {return new DcMotor[0]; }
    public double getRampingDistance() {return rampingDistance; }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public FunctionLibrary.Point getPosition() {
        return new FunctionLibrary.Point(x,y);
    }
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setPosition(FunctionLibrary.Point position) {
        x = position.x;
        y = position.y;
    }
    public void setPosition(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public void setRotation(double targetRotation) {
        rotationOffset = WrapAngleDegrees(targetRotation)-gyroRotation;
        worldRotation = WrapAngleDegrees(targetRotation);
    }
    public void addDeviation(FunctionLibrary.Point pointDeviation) {
        x += pointDeviation.x;
        y += pointDeviation.y;
    }
    public double getWorldRotation() {
        return worldRotation;
    }
    public double getRotationOffset() {
        return rotationOffset;
    }
    public double getGyroRotation() {
        return gyroRotation;
    }
}