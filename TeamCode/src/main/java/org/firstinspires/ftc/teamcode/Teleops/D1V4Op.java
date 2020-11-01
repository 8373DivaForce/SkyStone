package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4hardware;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;

@TeleOp
@Disabled
public class D1V4Op extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        D1V4hardware robot = new D1V4hardware(this,0,0,0);
        robot.dcOpenClose.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean fieldCentric = true;
        boolean leftStickButton = false;
        boolean rightStickButton = false;
        boolean xIsPressed = false;
        double dOffset = 0;
        waitForStart();
        while (opModeIsActive()) {
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
            else if(!gamepad1.right_stick_button && rightStickButton) rightStickButton = false;
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double dX;
            double dY;
            double rotation;
            if (fieldCentric) {
                double angle = Math.toDegrees(Math.atan2(y,x))+90;
                telemetry.addData("angle:", angle);
                angle = angle - GetYaw(0,robot.imu) - dOffset;
                telemetry.addData("adjustedAngle", angle);
                double hyp = Math.sqrt((x*x) + (y*y));
                telemetry.addData("Rotation", robot.getWorldRotation());
                telemetry.addData("hyp", hyp);

                rotation = gamepad1.right_stick_x;
                dX = Math.cos(Math.toRadians(angle))*hyp * -1;
                dY = Math.sin(Math.toRadians(angle))*hyp;
            } else {
                dY = gamepad1.left_stick_x;
                dX = gamepad1.left_stick_y;
                rotation = gamepad1.right_stick_x;
            }

            robot.move(dX,dY,rotation,1);
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.update();
            if (gamepad1.dpad_left) {
                robot.csRight.setPower(-1);
                robot.csLeft.setPower(-1);
            } else if(gamepad1.dpad_right) {
                robot.csRight.setPower(1);
                robot.csLeft.setPower(1);
            } else {
                robot.csRight.setPower(0);
                robot.csLeft.setPower(0);
            }
            if (gamepad1.right_trigger > 0 && !robot.upperLimitSwitch.isPressed()) {
                robot.dcUpDown1.setPower(gamepad1.right_trigger);
                robot.dcUpDown2.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0 && !robot.lowerLimitSwitch.isPressed()) {
                robot.dcUpDown1.setPower(-gamepad1.left_trigger);
                robot.dcUpDown2.setPower(-gamepad1.left_trigger);
            } else {
                robot.dcUpDown1.setPower(0);
                robot.dcUpDown2.setPower(0);
            }
            if (gamepad1.y) {
                robot.dcInOut.setPower(0.5);
            } else if (gamepad1.a) {
                robot.dcInOut.setPower(-0.5);
            } else {
                robot.dcInOut.setPower(0);
            }

            if (gamepad1.left_bumper) {
                robot.dcOpenClose.setPower(-1);
            } else if (gamepad1.right_bumper) {
                robot.dcOpenClose.setPower(1);
            } else {
                robot.dcOpenClose.setPower(0);
            }

            telemetry.update();
        }
    }
}
