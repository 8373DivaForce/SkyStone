package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

@Autonomous
@Disabled
public class TestRotPID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        D1V4hardware robot = new D1V4hardware(this,0);
        AutoFunctions auto = new AutoFunctions(robot);
        waitForStart();
        while (auto.rotPID(180,1,1,10) > -1 && opModeIsActive()) {
            telemetry.addData("angle:", robot.getWorldRotation());
            telemetry.update();
        };


    }
}
