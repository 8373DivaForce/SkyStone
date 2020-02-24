package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

@Autonomous
public class purePursuitComparison extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,new Point(0,0),0);
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        Point[] points = new Point[6];
        points[0] = new Point(0,0);
        points[1] = new Point(0,110);
        points[2] = new Point(-62,110);
        points[3] = new Point( -62,10);
        points[4] = new Point(-20, 10);
        points[5] = new Point(-0, 0);
        int nSwitch = 0;
        int result = 0;
        while (opModeIsActive()) {
            switch (nSwitch) {
                default:
                    result = auto.gotoPosition(points[nSwitch],1,1,0);
                    if (result <0) nSwitch++;
                    break;
                case 6:
                    break;
            }
        }

    }
}
