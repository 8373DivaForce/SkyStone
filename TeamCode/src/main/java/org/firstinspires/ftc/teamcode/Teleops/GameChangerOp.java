package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;

@TeleOp
public class GameChangerOp extends LinearOpMode {

    //constants for teleop run
    int magazineTurnRate = 280;
    double speedMultiplier = 1;
    double shooterPow = 0.73;
    int CAMPos = 0;
    double[] camPoses = {
            0.96,
            0.52
    };
    String[] camPosNames = {
            "High Goal",
            "Power Shot"
    };
    @Override
    public void runOpMode() throws InterruptedException {
        //initialization of variables like the robot and controller button states.
        GameChangerBotHardware robot = new GameChangerBotHardware(this,0,0,0);
        boolean aIsPressed = false;
        boolean dpadDownIsPressed = false;
        boolean xIsPressed = false;
        boolean fieldCentric = false;
        boolean leftBumperIsPressed = false;
        boolean rightBumperIsPressed = false;
        boolean yIsPressed = false;
        boolean bIsPressed = false;
        boolean leftTriggerIsPressed = false;
        boolean rightTriggerIsPressed = false;
        boolean leftStickIsPressed = false;
        boolean rightStickIsPressed = false;
        boolean dpadUpPressed = false;
        int wobblePivotPos = 0;
        double dOffset = 0;
        int lastShooterPos = 0;
        double lastTime = getRuntime();
        double curVelocity = 0;
        boolean goingDown = false;
        //setting up the magazine and shooter motors for operation
        robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setting initial CAM position
        robot.CAM.setPosition(0.57);
        //waits for the teleop to start
        waitForStart();
        //sets starting motor states
        robot.shooter.setPower(-shooterPow);
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        robot.magazine.setTargetPosition(0);
        robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.magazine.setPower(1);
        DcMotorEx shooter = (DcMotorEx)robot.shooter;
        //main teleop loop
        while (opModeIsActive()) {
            telemetry.addData("Front1", robot.dcBackLeft.getCurrentPosition());
            telemetry.addData("Front2", robot.dcBackRight.getCurrentPosition());
            telemetry.addData("left", robot.dcFrontLeft.getCurrentPosition());
            //gets position for testing purposes
            FunctionLibrary.Point position = robot.getPosition();
            telemetry.addData("position", position.x + ", " + position.y);
            telemetry.addData("rotation", robot.getWorldRotation());
            //gets the controller x, y, and rotation movement from sticks and scales them
            double x = FunctionLibrary.scaleInput(gamepad1.left_stick_x);
            double y = FunctionLibrary.scaleInput(gamepad1.left_stick_y);
            double dX;
            double dY;
            double rotation = FunctionLibrary.scaleInput(gamepad1.right_stick_x);
            //returns whether or not fieldCentric is enabled.
            telemetry.addData("fieldCentric", fieldCentric);
            //checks leftBumper to toggle field centric on and off
            if (!leftBumperIsPressed && gamepad1.left_bumper) {
                if (fieldCentric) {
                    fieldCentric = false;
                } else fieldCentric = true;
                dOffset = -GetYaw(0,robot.imu);
                leftBumperIsPressed = true;
            }
            else if (!gamepad1.left_bumper && leftBumperIsPressed) leftBumperIsPressed = false;
            //checks the rightBumper to reset which direction is forwards for field centric
            if (!rightBumperIsPressed && gamepad1.right_bumper) {
                dOffset = -GetYaw(0,robot.imu);
                rightBumperIsPressed = true;
            }
            else if(!gamepad1.right_bumper && rightBumperIsPressed) rightBumperIsPressed = false;
            //translates the values for field centric if it is enabled
            if (fieldCentric) {
                //gets the vector angle for the controller inputs in degress
                double angle = Math.toDegrees(Math.atan2(y,x))+90;
                telemetry.addData("angle:", angle);
                //translates the vector inputs from the controller in relation to the robot angle
                angle = angle - GetYaw(0,robot.imu) - dOffset;
                telemetry.addData("adjustedAngle", angle);
                //finds the hypot of that vector to use trig on
                double hyp = Math.sqrt((x*x) + (y*y));
                telemetry.addData("Rotation", robot.getWorldRotation());
                telemetry.addData("hyp", hyp);
                //finds the x and y movement from the translated vector
                dX = Math.cos(Math.toRadians(angle))*hyp * -1;
                dY = Math.sin(Math.toRadians(angle))*hyp;
            } else {
                dY = x;
                dX = y;
            }
            //enabled/disables the intake in the forwards direction when a is pressed
            if (!aIsPressed && gamepad1.a) {
                double intake = robot.intakeRD.getPower();
                double power = intake == 1 ? 0 : 1;
                //intake and deflector motors are linked
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                aIsPressed = true;
            } else if(aIsPressed && !gamepad1.a) {
                aIsPressed = false;
            }
            //enables/disables the intake in the backwards direction when x is pressed
            if (!xIsPressed && gamepad1.x) {
                double intake = robot.intakeRD.getPower();
                double power = intake == -1 ? 0 : -1;
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                xIsPressed = true;
            } else if (xIsPressed && !gamepad1.x) {
                xIsPressed = false;
            }
            //moves the magazine forwards the number of encoder ticks specified in the magazineTurnRate variable
            if (!yIsPressed && gamepad1.y) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos < 0 ? robot.magazine.getCurrentPosition()+magazineTurnRate : targetPos+magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                yIsPressed = true;
            } else if(yIsPressed && !gamepad1.y) {
                yIsPressed = false;
            }
            //moves the magazine backwards the number of encoder ticks specified in the magazineTurnRate variable
            if (!bIsPressed && gamepad1.b) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos > 0 ? robot.magazine.getCurrentPosition()-magazineTurnRate : targetPos-magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bIsPressed = true;
            } else if (bIsPressed && !gamepad1.b) {
                bIsPressed = false;
            }

            //enables/disables the shooter in the backwards direction
            if (!leftTriggerIsPressed && gamepad1.left_trigger > 0.1) {
                double shooterPower = robot.shooter.getPower();
                robot.shooter.setPower(shooterPower == 0 || shooterPower == -shooterPow ? shooterPow : 0);
                leftTriggerIsPressed = true;
            } else if(leftTriggerIsPressed && !(gamepad1.left_trigger > 0)) {
                leftTriggerIsPressed = false;
            }
            //enables/disables the shooter in the forwards direction
            if (!rightTriggerIsPressed && gamepad1.right_trigger > 0.1) {
                double shooterPower = robot.shooter.getPower();
                robot.shooter.setPower(shooterPower == 0 || shooterPower == shooterPow ? -shooterPow : 0);
                rightTriggerIsPressed = true;
            } else if (rightTriggerIsPressed && !(gamepad1.right_trigger > 0.1)) {
                rightTriggerIsPressed = false;
            }
            //iterates through the positions for the wobble goal mechanism
            if (gamepad1.right_stick_button && !rightStickIsPressed) {

                if (!goingDown) wobblePivotPos = (wobblePivotPos+1)%3;
                else wobblePivotPos = wobblePivotPos == 0 ? 2 : wobblePivotPos-1;
                if (wobblePivotPos == 2) goingDown = true;
                else if (wobblePivotPos == 0) goingDown = false;
                rightStickIsPressed = true;
            } else if (!gamepad1.right_stick_button){
                rightStickIsPressed = false;
            }
            //translates the specified position to it's servo position.
            robot.wobblePivot.setPosition(wobblePivotPos == 0 ? 0 : wobblePivotPos == 1 ? 0.55 : 1);
            //opens/closes the grabber on the wobble goal mechanism when the left stick button is pressed
            if (gamepad1.left_stick_button && !leftStickIsPressed) {
                robot.wobbleGrab.setPosition(robot.wobbleGrab.getPosition() == 0 ? 1 : 0);
                leftStickIsPressed = true;
            } else if (!gamepad1.left_stick_button){
                leftStickIsPressed = false;
            }
            //enables/disables slow mode which halves the controller input for the drive motors to allow for finer control
            if (gamepad1.dpad_down && !dpadDownIsPressed) {
                speedMultiplier = speedMultiplier == 0.25 ? 1 : 0.25;
                dpadDownIsPressed = true;
            } else if (!gamepad1.dpad_down && dpadDownIsPressed) {
                dpadDownIsPressed = false;
            }
            //cycles through the preset CAM positions for the shooter
            if (gamepad1.dpad_up && !dpadUpPressed) {
                CAMPos = (CAMPos+1)%2;
                robot.CAM.setPosition(camPoses[CAMPos]);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up && dpadUpPressed) {
                dpadUpPressed = false;
            }
            //allows for manual control of the CAM angle using dpad_left and dpad_right
            robot.CAM.setPosition(robot.CAM.getPosition()+((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0))*getRuntime());
            resetStartTime();
            robot.move(dX*speedMultiplier,dY*speedMultiplier,rotation*speedMultiplier,1);
            //logs current variable states
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.addData("pivotPos", wobblePivotPos);
            double curCAMPos = Math.round(robot.CAM.getPosition()*100)/100;
            telemetry.addData("CAMPos", robot.CAM.getPosition());
            telemetry.addData("Shooter", shooter.getVelocity());
            telemetry.update();

        }
    }
}
