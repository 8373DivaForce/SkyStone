package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.openftc.revextensions2.RevBulkData;

@Autonomous
public class Latency_Tester extends LinearOpMode {
    private static class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //NewKissHardware robot = new NewKissHardware(this);
        RevBulkData revBulkData;
        int iterations = 1000;
        double[] time = new double[iterations+1];
        double[] time2 = new double[iterations+1];
        double[] time3 = new double[iterations+1];
        //telemetry.addData("Firmware Version Back Hub",robot.expansionHub.getFirmwareVersion());
       // telemetry.addData("Firmware Version Front Hub",hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 5").getFirmwareVersion());
        telemetry.update();
        waitForStart();
        time[0] = getRuntime();
        /*
        for (int i = 0; i < iterations; i++) {
            GetYaw(0,robot.imu);
            robot.dcFrontRight.getCurrentPosition();
            robot.dcFrontLeft.getCurrentPosition();
            robot.dcBackLeft.getCurrentPosition();
            robot.dcBackRight.getCurrentPosition();
            time[i+1] = getRuntime();
        }
        time2[0] = getRuntime();
        for (int i = 0; i < iterations; i++) {
            GetYaw(0,robot.imu);
            revBulkData = robot.expansionHub.getBulkInputData();
            time2[i+1] = getRuntime();
        }
        */
        //robot.imu.close();
        LynxModule module = hardwareMap.getAll(LynxModule.class).iterator().next();
        LynxEmbeddedIMU imu = new LynxEmbeddedIMU(new BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV2(hardwareMap.appContext, module, 0), true
        ));
        imu.initialize();
        time3[0] = getRuntime();
        for (int i = 0; i < iterations; i++) {
            Log.i("Angular orientation: ",imu.getAngularOrientation().toString());
            time3[i+1] = getRuntime();
        }
        imu.close();


        double maxRun1 = 0;
        double minRun1 = 999999;
        double average1 = 0;
        double maxRun2 = 0;
        double minRun2 = 99999999;
        double average2 = 0;
        double maxRun3 = 0;
        double minRun3 = 99999999;
        double average3 = 0;

        for (int i = 0; i < iterations; i++) {
            double latency1 = ((int)((time[i+1] - time[i])*100000))/100;
            double latency2 = ((int)((time2[i+1] - time2[i])*100000))/100;
            double latency3 = ((int)((time3[i+1] - time3[i])*100000))/100;
            maxRun1 = Math.max(maxRun1, latency1);
            maxRun2 = Math.max(maxRun2, latency2);
            maxRun3 = Math.max(maxRun3, latency3);

            minRun1 = Math.min(minRun1, latency1);
            minRun2 = Math.min(minRun2, latency2);
            minRun3 = Math.min(minRun3, latency3);

            average1 += latency1;
            average2 += latency2;
            average3 += latency3;
        }
        average1 /= iterations;
        average2 /= iterations;
        average3 /= iterations;
        while (opModeIsActive()) {
            telemetry.addData("Run1", "Average: " + average1 + ", Min: " + minRun1 + ", Max: " + maxRun1);
            telemetry.addData("Run2", "Average: " + average2 + ", Min: " + minRun2 + ", Max: " + maxRun2);
            telemetry.addData("Run3", "Average: " + average3 + ", Min: " + minRun3 + ", Max: " + maxRun3);
            telemetry.update();
        }
    }
}
