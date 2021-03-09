package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;

@Autonomous(group = "Calibration")
public class TurnCalibrator extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //motor setup
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotor front1 = backLeft;
        int front1Multi = 1;
        DcMotor front2 = backRight;
        double front2Multi = 1*(0.8978179);
        DcMotor left = frontLeft;
        int leftMulti = 1;

        BNO055IMU.Parameters BNparameters = new BNO055IMU.Parameters();
        BNparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNparameters.loggingEnabled = true;
        BNparameters.loggingTag = "IMU";
        BNparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //define the IMU from the hardwaremap
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        //initialize the IMU
        imu.initialize(BNparameters);

        waitForStart();
        int nState = 0;
        double ticksPerAngle = 0;
        double anglePerTick = 0;
        double angle;
        while (opModeIsActive()) {
            switch(nState) {
                case 0:
                    frontLeft.setPower(-0.5);
                    frontRight.setPower(0.5);
                    backLeft.setPower(-0.5);
                    backRight.setPower(0.5);
                    nState++;
                    break;
                case 1:
                    angle = FunctionLibrary.GetYaw(0,imu);
                    if (Math.abs(angle-90) < 5) nState++;
                    break;
                case 2:
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    resetStartTime();
                    nState++;
                    break;
                case 3:
                    if (getRuntime() >= 1) nState++;
                    break;
                case 4:
                    angle = FunctionLibrary.GetYaw(0,imu);
                    ticksPerAngle = (front1.getCurrentPosition()*front1Multi)/angle;
                    anglePerTick = (left.getCurrentPosition()*leftMulti)/angle;
                    nState++;
                    break;
                case 5:
                    telemetry.addData("ticksPerAngle", ticksPerAngle);
                    telemetry.addData("anglePerTick", anglePerTick);
                    telemetry.update();
                    break;
            }
        }

    }
}

