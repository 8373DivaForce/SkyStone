package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;
import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;

@Autonomous
public class taskHandlerTest extends LinearOpMode {

    public DcMotor[] group(DcMotor ... motors) {
        return motors;
    }
    public int[] group(int ... ints) {
        return ints;
    }
    public double[] group(double...doubles) {
        return doubles;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        OldKissBotHArdware robot = new OldKissBotHArdware(this, 0, 0,0);
        taskHandler handler = new taskHandler();
        handler.addTask(new baseTasks.move(new Point(24,24),0,1,1,1000));
        handler.addTask(new baseTasks.move(new Point(0,0),1,1,1000));
        handler.addTask(new baseTasks.move(new Point(24,24),0,1,1,1000));
        handler.addTask(new baseTasks.move(new Point(0,0),0,1,1,1000));
        handler.addTask(new baseTasks.motorMovement(group(robot.dcBackLeft, robot.dcFrontRight), group(10000,10000),1,100,5000));
        waitForStart();
        while (opModeIsActive()) {
            handler.loop(robot);
            telemetry.addData("Task: ", handler.curTask);
            telemetry.update();
        }

    }
}
