package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissHardware;
import org.openftc.revextensions2.RevBulkData;

@TeleOp
@Disabled
public class NewKissOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NewKissHardware robot = new NewKissHardware(this,0,new FunctionLibrary.Point(0,0));
        waitForStart();
        while (opModeIsActive()) {
            robot.move(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x, 1);
            telemetry.addData("rotation", robot.getWorldRotation());
            FunctionLibrary.Point pos = robot.getPosition();
            telemetry.addData("position", pos.x + ", " + pos.y);
            telemetry.addData("Ticks per rev", robot.dcFrontLeft.getMotorType().getTicksPerRev());
            RevBulkData revBulkData = robot.bulkData;
            if (revBulkData != null) {
                telemetry.addData("Front Left", revBulkData.getMotorVelocity(robot.dcFrontLeft));
                telemetry.addData("Front Right", revBulkData.getMotorVelocity(robot.dcFrontRight));
                telemetry.addData("Back Left", revBulkData.getMotorVelocity(robot.dcBackLeft));
                telemetry.addData("BackRight", revBulkData.getMotorVelocity(robot.dcBackRight));
            }
            telemetry.update();
        }
    }
}
