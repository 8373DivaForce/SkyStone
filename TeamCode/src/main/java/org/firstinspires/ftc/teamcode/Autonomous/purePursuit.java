package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissHardware;

import java.util.ArrayList;

@Autonomous
public class purePursuit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewKissHardware robot = new NewKissHardware(this,0,new Point(0,0));
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        Point[] points = new Point[5];
        points[0] = new Point(0,0);
        points[1] = new Point(0,110);
        points[2] = new Point(-62,110);
        points[3] = new Point( -62,10);
        points[4] = new Point(-20, 10);
        Point[] points2 = new Point[2];
        points2[0] = new Point(-20,10);
        points2[1] = new Point(0,10);
        int nSwitch = 0;
        double result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                case 0:
                    result = auto.purePursuit(points,10,0.5, 5, 0,telemetry);
                    telemetry.addData("lastRes", result);
                    if (result < 0) nSwitch++;
                    break;
                case 1:
                    result = auto.purePursuit(points,10,0.5,1,0,telemetry);
                    if(result< 0) nSwitch++;
                    break;
                case 2:
                    result = auto.rotPID(0,0.5,5,5);
                    if(result<0) nSwitch++;
                    break;
                case 3:
                    result = auto.gotoPosition(new Point(0,0), 0.5, 1, 0);
                    if (result < 0) nSwitch++;
                    break;
            }
            telemetry.addData("state", nSwitch);
            telemetry.update();
        }

    }
}
