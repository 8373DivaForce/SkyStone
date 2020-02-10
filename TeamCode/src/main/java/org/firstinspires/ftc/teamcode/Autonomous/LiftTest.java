package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

public class LiftTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,0,0,0);
        FunctionLibrary.motorMovement lift = new FunctionLibrary.motorMovement(0,robot.dcLift, robot.dcLift2);
        lift.limits(robot.upperLimitSwitch, robot.lowerLimitSwitch);
        waitForStart();
        int nSwitch = 0;
        switch (nSwitch) {
            case 0:
                double result = lift.move_using_encoder(1000, 0.5, 2, 50, false);
                if (result < 0) nSwitch++;
                break;
        }
    }
}
