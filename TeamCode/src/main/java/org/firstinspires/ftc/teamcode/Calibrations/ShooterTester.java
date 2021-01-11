package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;
import org.firstinspires.ftc.teamcode.Libraries.functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;

@Autonomous(group = "Calibration")
public class ShooterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor shooter = hardwareMap.dcMotor.get("frontRight");
        shooter.setPower(0);

        resetStartTime();
        while (!isStopRequested() && getRuntime() < 5) {
            shooter.setPower(getRuntime()/5);
        }
        shooter.setPower(1);
        resetStartTime();
        while (!isStopRequested() && getRuntime() < 5) { }

        int nIterations = 100;

        double average = 0;
        double minValue = 99999999;
        double maxValue = -99999999;
        for (int i = 0; i < nIterations && !isStopRequested(); i++) {
            double curVelocity = ((DcMotorEx)shooter).getVelocity();
            average += curVelocity;
            minValue = Math.min(minValue,curVelocity);
            maxValue = Math.max(maxValue, curVelocity);
        }
        average /= nIterations;
        double timeToSpeedUp = 0;
        int iterations = 0;
        boolean calculating = false;
        while (!isStopRequested()) {
            double curVelocity = ((DcMotorEx)shooter).getVelocity();
            telemetry.addData("Average", average);
            telemetry.addData("Min Normal Speed", minValue);
            telemetry.addData("Max Normal Speed", maxValue);
            telemetry.addData("Range", maxValue-minValue);
            telemetry.addData("Current Velocity", curVelocity);
            if (curVelocity < minValue && !calculating) {
                resetStartTime();
                calculating = true;
            } if (calculating && curVelocity > minValue) {
                calculating = false;
                timeToSpeedUp += getRuntime();
                iterations++;
            }
            telemetry.addData("Average Wind Up Time", timeToSpeedUp/iterations);
            telemetry.update();
        }
    }
}
