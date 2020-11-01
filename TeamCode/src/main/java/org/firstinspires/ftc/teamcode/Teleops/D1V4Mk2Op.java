package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware_Maps.D1V4Mk2hardware;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;
import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.scaleInput;

@TeleOp(group = "A")
@Disabled
public class D1V4Mk2Op extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize the D1V4Mk2Hardware map
        D1V4Mk2hardware robot = new D1V4Mk2hardware(this,0,0,0,"Right Webcam");
        robot.dcOpenClose.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setup initial variables for operation
        boolean fieldCentric = false;
        boolean leftStickButton = false;
        boolean rightStickButton = false;
        boolean leftDPadPressed = false;
        boolean rightDPadPressed = false;
        boolean xPressed = false;
        double dOffset = 0;

        double powerMultiplier = 1;
        double slowSpeed = 0.5;
        //wait for the opmode to start
        waitForStart();
        //main program loop
        while (opModeIsActive()) {

            if (gamepad1.x && !xPressed) {
                if (powerMultiplier == 1) {
                    powerMultiplier = slowSpeed;
                } else {
                    powerMultiplier = 1;
                }
            } else if (!gamepad1.x && xPressed) xPressed = false;

            telemetry.addData("fieldCentric", fieldCentric);
            if (powerMultiplier == 1) telemetry.addData("Slow", "No");
            else telemetry.addData("Slow", "Yes");
            //allow the disabiling/renenabling of field centric
            if (!leftStickButton && gamepad1.left_stick_button) {
                if (fieldCentric) {
                    fieldCentric = false;
                } else fieldCentric = true;
                dOffset = -GetYaw(0,robot.imu);
                leftStickButton = true;
            }
            else if (!gamepad1.left_stick_button && leftStickButton) leftStickButton = false;
            //allow the reset of field centric rotation
            if (!rightStickButton && gamepad1.right_stick_button) {
                dOffset = -GetYaw(0,robot.imu);
                rightStickButton = true;
            }
            else if(!gamepad1.right_stick_button && rightStickButton) rightStickButton = false;
            //set the x and y movement as what's on the gamepad
            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            //make sure the values aren't too small
            x = Math.abs(x) < 0.05 ? 0 : scaleInput(x)*powerMultiplier;
            y = Math.abs(y) < 0.05 ? 0 : scaleInput(y)*powerMultiplier;
            //set the rotation to what is being read from the gamepad
            double rotation = gamepad1.right_stick_x;
            //make sure it isn't too low
            rotation = Math.abs(rotation) < 0.05 ? 0 : scaleInput(rotation)*powerMultiplier;
            //initialize translated x and y based off of field centric
            double dX;
            double dY;
            //check if field centric is enabled
            if (fieldCentric) {
                //find the angle of our movement vector
                double angle = Math.toDegrees(Math.atan2(y,x))+90;
                telemetry.addData("angle:", angle);
                //translate it by our current gyro angle
                angle = angle - GetYaw(0,robot.imu) - dOffset;
                telemetry.addData("adjustedAngle", angle);
                //find the hypot of the angle
                double hyp = Math.sqrt((x*x) + (y*y));
                telemetry.addData("Rotation", robot.getWorldRotation());
                telemetry.addData("hyp", hyp);
                //get the new x and y based off of our rotation
                dX = Math.cos(Math.toRadians(angle))*hyp * -1;
                dY = Math.sin(Math.toRadians(angle))*hyp;
            } else {
                //transfer the x and y to the newer ones
                dY = x;
                dX = y;
            }
            //feed the values to our movement function
            robot.move(dX,dY,rotation,1);
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.update();
            //allow the scissor lift to move up and down
            //if the upperlimit switch is being pressed, don't allow the lift to move up
            if (gamepad1.right_trigger > 0 && !robot.upperLimitSwitch.isPressed()) {
                robot.dcLift.setPower(gamepad1.right_trigger);
                robot.dcLift2.setPower(gamepad1.right_trigger);
            //if the lowerlimit switch is being pressed, don't allow the lift to move down
            } else if (gamepad1.left_trigger > 0 && !robot.lowerLimitSwitch.isPressed()) {
                robot.dcLift.setPower(-gamepad1.left_trigger);
                robot.dcLift2.setPower(-gamepad1.left_trigger);
            //if nothing is being pressed, reset the values to 0
            } else {
                robot.dcLift.setPower(0);
                robot.dcLift2.setPower(0);
            }
            //allow the inout mech to move in and out based off of y and a
            if (gamepad1.y) {
                robot.dcInOut.setPower(0.5);
            } else if (gamepad1.a) {
                robot.dcInOut.setPower(-0.5);
            } else {
                robot.dcInOut.setPower(0);
            }

            //allow the gripper to open and close
            if (gamepad1.left_bumper) {
                robot.dcOpenClose.setPower(-1);
            } else if (gamepad1.right_bumper) {
                robot.dcOpenClose.setPower(1);
            } else {
                robot.dcOpenClose.setPower(0);
            }
            //allow the left dpad button to toggle the position of the left hook servo
            if (gamepad1.dpad_left && !leftDPadPressed) {
                if (robot.sLStoneHook.getPosition() == 0) {
                    robot.sLStoneHook.setPosition(0.8);
                } else {
                    robot.sLStoneHook.setPosition(0);
                }
                leftDPadPressed = true;
            } else if(leftDPadPressed && !gamepad1.dpad_left) {
                leftDPadPressed = false;
            }
            //allow the right dpad button to toggle the position of the right hook servo
            if (gamepad1.dpad_right && !rightDPadPressed) {
                if (robot.sRStoneHook.getPosition() == 0) {
                    robot.sRStoneHook.setPosition(0.7);
                } else {
                    robot.sRStoneHook.setPosition(0);
                }
                rightDPadPressed = true;
            } else if(rightDPadPressed && !gamepad1.dpad_right) {
                rightDPadPressed = false;
            }
            telemetry.update();
        }
    }
}
