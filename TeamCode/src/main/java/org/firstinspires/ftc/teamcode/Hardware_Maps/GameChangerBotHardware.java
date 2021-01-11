package org.firstinspires.ftc.teamcode.Hardware_Maps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.opencv.core.Scalar;

import java.io.File;

import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class GameChangerBotHardware extends RobotConstructor {
    //setup initial parameters that are provided to the parent robotconstructor class
    public static final String VuforiaKey =
            "AW7ToAj/////AAABmcZ9RLZ3tUhClKOp3feoyDVjA4MD06H8ulSOPwGXzZJr7gNfTHtYBvWN9wxei4kahK3/60QQk6t+SpYL44+w/RKvX0Yk8bl4wwhljb1cT8509LQsZnaCu+UH6NeGNgDh7fcPcKlEdXlw5eB62IF/1xzfeQ//vH9pD4Ihu7XhaZzv8wD827zQWT+yrdxxfFEvTR7xWLIj23JqgI+t4glIuAmQPKBHXGTHDSXyr5uQbjqxCNJlkAhceGETf1RDBURZ2v3KGIqC3SVVV1ixlUMSGL9QqAzEGPHT2nF0nK4zt+WdsetLdTniZLkwr1hdn4vvzbH8tAbdfV/eeNWF+GtJmHtHjOk3exEvMrH+ZflXUIoY";
    private static String Webcamname;
    private static double wheelDiameter = 3.93701;
    private static double dKp = 0.05;
    private static double minMoveSpeed = 0.1;
    public static float CameraForwardDisplacement;
    public static float CameraLeftDisplacement;
    public static float CameraVerticalDisplacement;
    private static float rampingDistance = 12;
    private static int odometryUpdateRate = 24;

    public final double inchesPerTickX;
    public final double inchesPerTickY;

    //initialize the variables for the hardware devices
    public final DcMotor dcFrontLeft;
    public final DcMotor dcFrontRight;
    public final DcMotor dcBackLeft;
    public final DcMotor dcBackRight;

    public final DcMotor intakeRD;
    public final DcMotor magazine;
    public final DcMotor shooter;
    public final DcMotor CAM;




    private final static String name = "Kissbot";
    //setup the constructor function
    public GameChangerBotHardware(LinearOpMode opMode, double rotation, String camera) {
        //provide the opMode given on creation as well as the variables defined above
        super(opMode, name, wheelDiameter, dKp, minMoveSpeed,rampingDistance, 0, CameraLeftDisplacement, CameraVerticalDisplacement, VuforiaKey, odometryUpdateRate);
        //save the hardware map from the opMode
        HardwareMap hMap = opMode.hardwareMap;

        //set the variables to their corresponding hardware device
        dcFrontLeft = hMap.dcMotor.get("frontLeft");
        dcFrontRight = hMap.dcMotor.get("frontRight");
        dcBackLeft = hMap.dcMotor.get("backLeft");
        dcBackRight = hMap.dcMotor.get("backRight");

        intakeRD = hMap.dcMotor.get("intakeRD");
        magazine = hMap.dcMotor.get("magazine");
        shooter = hMap.dcMotor.get("shooter");
        CAM = hMap.dcMotor.get("CAM");

        //setup the directions the devices need to operate in
        dcFrontRight.setDirection(DcMotor.Direction.REVERSE);
        dcBackRight.setDirection(DcMotor.Direction.REVERSE);

        //make sure none of the devices are running
        dcFrontLeft.setPower(0);
        dcFrontRight.setPower(0);
        dcBackLeft.setPower(0);
        dcBackRight.setPower(0);

        intakeRD.setPower(0);
        magazine.setPower(0);
        shooter.setPower(0);
        CAM.setPower(0);

        //Reset the encoders on every motor
        dcFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CAM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set them to run without the encoders by default
        dcFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        dcBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CAM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //initialize a variable useful in the odometry function
        double tempInchPerTick = (1/dcFrontLeft.getMotorType().getTicksPerRev())*getWheelCircumfrance();
        // old inchesPerTickX = tempInchPerTick*-0.92307692307692307692307692307692;
        inchesPerTickX = tempInchPerTick*-1.36893203883;

        // old inchesPerTickY = tempInchPerTick*1.1483253588516746411483253588517;
        inchesPerTickY = tempInchPerTick*1.5;
        useOdometry = false;
        setRotation(rotation);
        initOdometry();
    }
    public GameChangerBotHardware(LinearOpMode opMode, double x, double y, double rotation) {
        this(opMode, rotation, null);
        setPosition(x,y);
        useOdometry = true;
    }
    public GameChangerBotHardware(LinearOpMode opMode, double x, double y, double rotation, String camera) {
        this(opMode, rotation, camera);
        setPosition(x,y);
        useOdometry = true;
    }
    public GameChangerBotHardware(LinearOpMode opMode, FunctionLibrary.Point startingPos, double rotation) {
        this(opMode, startingPos.x, startingPos.y, rotation);
    }
    public GameChangerBotHardware(LinearOpMode opMode, FunctionLibrary.Point startingPos, double rotation, String camera) {
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

        //add forwards motion
        double pFrontLeft= x + y;
        double pFrontRight = x - y;
        double pBackLeft= x - y;
        double pBackRight = x + y;

        //use the scaleDownValues function from the FunctionLibrary and give it the power as the max and all of the motors
        double[] powers = FunctionLibrary.scaleDownValues(power, pFrontLeft, pFrontRight, pBackLeft, pBackRight);

        //add left/right motion
        pFrontLeft = powers[0] - rotation;
        pFrontRight = powers[1] + rotation;
        pBackLeft = powers[2] - rotation;
        pBackRight = powers[3] + rotation;
        //scale down values again
        powers = FunctionLibrary.scaleDownValues(power, pFrontLeft, pFrontRight, pBackLeft, pBackRight);
        pFrontLeft = powers[0];
        pFrontRight = powers[1];
        pBackLeft = powers[2];
        pBackRight = powers[3];
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
    File colorLimits = AppUtil.getInstance().getSettingsFile("openCVColorLimits");
    public void toCSV(Scalar mins, Scalar maxes) {
        toCSV(mins.val,maxes.val);
    }
    public void toCSV(double[] mins, double[] maxes) {
        String output = "";
        output += mins[0] + ",";
        output += mins[1] + ",";
        output += mins[2] + ",";
        output += maxes[0] + ",";
        output += maxes[1] + ",";
        output += maxes[2];
        ReadWriteFile.writeFile(colorLimits,output);
    }
    public Scalar[] fromCSV() {
        if (!colorLimits.exists()) {
            Scalar[] scalars = new Scalar[]{new Scalar(0,0,0),new Scalar(255,255,255)};
            toCSV(scalars[0],scalars[1]);
            colorLimits = AppUtil.getInstance().getSettingsFile("openCVColorLimits");
            return scalars;
        }
        String input = ReadWriteFile.readFile(colorLimits);
        String[] vals = input.split(",");
        Scalar[] scalars = new Scalar[2];
        double[] pVals = new double[6];
        for (int i = 0; i < 6; i++) {
            pVals[i] = Double.parseDouble(vals[i]);
        }
        scalars[0] = new Scalar(pVals[0],pVals[1],pVals[2]);
        scalars[1] = new Scalar(pVals[3],pVals[4],pVals[5]);
        return scalars;
    }
    public void disableOdometry() {
        useOdometry = false;
    }
    public void enableOdometry() {
        useOdometry = true;
    }
}
