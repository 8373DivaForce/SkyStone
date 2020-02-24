package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

import java.util.ArrayList;

@Autonomous
public class purePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,new Point(0,0),0);
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        Point[] points = new Point[5];
        points[0] = new Point(0,0);
        points[1] = new Point(0,110);
        points[2] = new Point(-62,110);
        points[3] = new Point( -62,10);
        points[4] = new Point(-20, 10);
        int nSwitch = 0;
        double result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                case 0:
                    result = auto.purePursuit(points,10,1, 5, 0,telemetry);
                    telemetry.addData("lastRes", result);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = auto.gotoPosition(new Point(0,0), 1, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
            }
            telemetry.addData("state", nSwitch);
            telemetry.update();
        }

    }
}
