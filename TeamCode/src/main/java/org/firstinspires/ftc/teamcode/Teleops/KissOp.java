package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.Kisshardware;

import static org.firstinspires.ftc.teamcode.Functions.FunctionLibrary.GetYaw;

@TeleOp
public class KissOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Kisshardware robot = new Kisshardware(this,0,0,0);
        boolean fieldCentric = false;
        boolean leftStickButton = false;
        boolean rightStickButton = false;
        boolean xIsPressed = false;
        double dOffset = 0;
        waitForStart();
        while (opModeIsActive()) {
            FunctionLibrary.Point position = robot.getPosition();
            telemetry.addData("position", position.x + ", " + position.y);
            telemetry.addData("fieldCentric", fieldCentric);
            if (!leftStickButton && gamepad1.left_stick_button) {
                if (fieldCentric) {
                    fieldCentric = false;
                } else fieldCentric = true;
                dOffset = -GetYaw(0,robot.imu);
                leftStickButton = true;
            }
            else if (!gamepad1.left_stick_button && leftStickButton) leftStickButton = false;
            if (!rightStickButton && gamepad1.right_stick_button) {
                dOffset = -GetYaw(0,robot.imu);
                rightStickButton = true;
            }
            if (gamepad1.left_bumper) robot.sRStoneHook.setPosition(0);
            else if (gamepad1.right_bumper) robot.sRStoneHook.setPosition(1);

            else if(!gamepad1.right_stick_button && rightStickButton) rightStickButton = false;
            double x = FunctionLibrary.scaleInput(gamepad1.left_stick_x);
            double y = FunctionLibrary.scaleInput(gamepad1.left_stick_y);
            double dX;
            double dY;
            double rotation = FunctionLibrary.scaleInput(gamepad1.right_stick_x);
            if (fieldCentric) {
                double angle = Math.toDegrees(Math.atan2(y,x))+90;
                telemetry.addData("angle:", angle);
                angle = angle - GetYaw(0,robot.imu) - dOffset;
                telemetry.addData("adjustedAngle", angle);
                double hyp = Math.sqrt((x*x) + (y*y));
                telemetry.addData("Rotation", robot.getWorldRotation());
                telemetry.addData("hyp", hyp);

                dX = Math.cos(Math.toRadians(angle))*hyp * -1;
                dY = Math.sin(Math.toRadians(angle))*hyp;
            } else {
                dY = x;
                dX = y;
            }

            robot.move(dX,dY,rotation,1);
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.update();

        }
    }
}
