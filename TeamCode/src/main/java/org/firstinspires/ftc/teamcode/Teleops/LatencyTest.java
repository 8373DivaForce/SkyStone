package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.sql.Time;
import java.util.Calendar;
import java.util.Timer;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

@TeleOp
@Disabled
public class LatencyTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel echo = hardwareMap.digitalChannel.get("echo");
        DigitalChannel trig = hardwareMap.digitalChannel.get("trig");
        ThreadSpeed threadSpeed = new ThreadSpeed(trig,echo);
        Thread thread = new Thread(threadSpeed);
        waitForStart();
        echo.setState(false);
        echo.setMode(DigitalChannel.Mode.INPUT);
        while (opModeIsActive()) {
            telemetry.addData("Distance", threadSpeed.lastDistance);
            telemetry.addData("Iterations", threadSpeed.iterations);
            telemetry.addData("Scans", threadSpeed.callStated);
            telemetry.addData("Echo", threadSpeed.echoStarted);
            telemetry.addData("Echo2", echo.getState());
            telemetry.update();
        }
    }
}

class ThreadSpeed implements Runnable {
    public volatile double startTime = 0;
    public volatile boolean running = true;
    public volatile long iterations = 0;
    public volatile double latency = 0;
    public volatile int callStated = 0;
    public volatile DigitalChannel trig;
    public volatile DigitalChannel echo;
    boolean listen = false;
    public ThreadSpeed(DigitalChannel trig, DigitalChannel echo) {
        this.trig = trig;
        this.echo = echo;
    }
    boolean echoStarted = false;
    public volatile double lastDistance = 0;
    @Override
    public void run() {
        startTime = System.nanoTime();
        trig.setMode(DigitalChannel.Mode.OUTPUT);
        echo.setMode(DigitalChannel.Mode.INPUT);
        while (running) {
            if (listen) {
                if (echo.getState() && !echoStarted) {
                    startTime = System.nanoTime();
                    echoStarted = true;
                } else if (echoStarted && !echo.getState()) {
                    lastDistance = ((System.nanoTime()-startTime)/1000)/147;
                    echoStarted = false;
                    listen = false;
                    callStated++;
                }
            } else {
                iterations++;
            }
        }

    }
    public void sendSignal() {
        trig.setState(true);
        startTime = System.nanoTime();
        while (System.nanoTime()-startTime < 10000);
        trig.setState(false);
        listen = true;
    }
}
