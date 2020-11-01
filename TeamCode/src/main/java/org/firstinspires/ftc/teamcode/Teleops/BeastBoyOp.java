package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Hardware_Maps.BeastBoyHardware;

@TeleOp
public class BeastBoyOp extends LinearOpMode {
    //setup function for scaling the controller inputs
    private final double scaleInput(double x) {
        double direction = x/Math.abs(x);
        x = Math.abs(x);
        if (x >= 0.09) {
            return Math.pow(x,3)*direction;
        } else {
            return 0;
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize the robot without odometry
        BeastBoyHardware robot = new BeastBoyHardware(this,0,new FunctionLibrary.Point(0,0),false);

        waitForStart();
        //setup variables for buttons that toggle things
        boolean foundPressed = false;
        boolean foundation = false;
        boolean speedPressed = false;
        double speedMultiplier = 1;
        boolean intakePressed = false;
        boolean intake = false;
        boolean gatePressed = false;
        boolean gate = false;
        boolean gripperPressed = false;
        boolean gripper = true;
        double forward = 1;
        boolean forwardPressed = false;
        while (opModeIsActive()) {
            //control the dcLift based off of the left and right triggers
            robot.dcLift.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
            //check if the right bumper has just been pressed, if so, toggle the foundation hooks
            if (gamepad1.right_bumper && !foundPressed) {
                foundPressed = true;
                foundation = !foundation;
                if (foundation) {
                    robot.foundLeft.setPosition(1);
                    robot.foundRight.setPosition(1);
                } else {
                    robot.foundLeft.setPosition(0);
                    robot.foundRight.setPosition(0);
                }
            } else if (foundPressed && !gamepad1.right_bumper) foundPressed = false; //if the button is no longer being pressed,
            //allow it to recognize that

            //allow the gripper to be open up all the way
            if (gamepad1.b) {
                gripper = false;
                robot.gTopLeft.setPosition(1);
                robot.gTopRight.setPosition(1);
                robot.gBottomLeft.setPosition(1);
                robot.gBottomRight.setPosition(1);
            } else if (gamepad1.x && !gripperPressed) {//check if x has just been pressed, if so, toggle the gripper
                gripperPressed = true;
                gripper = !gripper;
                double position;
                if (gripper) {
                    position = 0.5;
                } else position = 0;
                robot.gTopLeft.setPosition(position);
                robot.gTopRight.setPosition(position);
                robot.gBottomLeft.setPosition(position);
                robot.gBottomRight.setPosition(position);
            } else if (gripperPressed && !gamepad1.x) gripperPressed = false;

            if (gamepad1.left_stick_button && !speedPressed) { //Toggle the speed setting
                speedPressed = true;
                if (speedMultiplier == 0.5) speedMultiplier = 1;
                else speedMultiplier = 0.5;
            } else if (!gamepad1.left_stick_button && speedPressed) speedPressed = false;

            if ((gamepad1.dpad_left) && !intakePressed) { //toggle the intake
                intake = !intake;
                intakePressed = true;

                if (intake && gamepad1.dpad_left) {
                    robot.dcIntakeLeft.setPower(-1);
                    robot.dcIntakeRight.setPower(-1);
                } else {
                    robot.dcIntakeLeft.setPower(0);
                    robot.dcIntakeRight.setPower(0);
                }
            } else if (!gamepad1.left_bumper && !gamepad1.dpad_left && intakePressed) intakePressed = false;
            if (gamepad1.left_bumper && !gamepad1.dpad_left) {
                robot.dcIntakeLeft.setPower(1);
                robot.dcIntakeRight.setPower(1);
            } else if (!gamepad1.dpad_left) {
                robot.dcIntakeRight.setPower(0);
                robot.dcIntakeLeft.setPower(0);
            }
            if (gamepad1.dpad_down && !gatePressed) { //toggle the gate
                gate = !gate;
                gatePressed = true;
                if (gate) robot.gate.setPosition(1);
                else robot.gate.setPosition(0);
            } else if (!gamepad1.dpad_down && gatePressed) gatePressed = false;
            if (gamepad1.right_stick_button && !forwardPressed) {
                forwardPressed = true;
                forward *= -1;
            } else if (!gamepad1.right_stick_button && forwardPressed) forwardPressed = false;
            //take the gamepad inputs and scale them
            double x = scaleInput(gamepad1.left_stick_x);
            double y = scaleInput(gamepad1.left_stick_y);
            double theta = scaleInput(gamepad1.right_stick_x);
            //tell the robot to move based off of the inputs
            robot.move(y*forward,x*forward,theta,speedMultiplier);
            //relay information to the driver
            if (speedMultiplier == 1) {
                telemetry.addData("Slow", "NO");
            } else telemetry.addData("Slow", "YES");
            if (forward == 1) telemetry.addData("Forward", "Intake");
            else telemetry.addData("Forward", "Gripper");
            telemetry.update();

        }


    }
}
