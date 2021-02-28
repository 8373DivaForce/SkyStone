package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.LED.LEDRiver;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;

@TeleOp
public class GameChangerOp extends LinearOpMode {

    boolean intake = false;
    boolean isYPressed = false;
    boolean isAPressed = false;
    int magazineTurnRate = 250;
    double speedMultiplier = 1;
    boolean dpadDownIsPressed = false;
    double shooterPow = 0.72;
    int CAMPos = 0;
    double[] camPoses = {
            0.96,
            0.67
    };
    String[] camPosNames = {
            "High Goal",
            "Power Shot"
    };
    @Override
    public void runOpMode() throws InterruptedException {
        GameChangerBotHardware robot = new GameChangerBotHardware(this,0,0,0);
        boolean fieldCentric = false;
        boolean leftStickButton = false;
        boolean rightStickButton = false;
        boolean xIsPressed = false;
        boolean bIsPressed = false;
        boolean leftBumperIsPressed = false;
        boolean rightBumperIsPressed = false;
        boolean dpadLeftIsPressed = false;
        boolean dpadRightIsPressed = false;
        boolean dpadUpPressed = false;
        int wobblePivotPos = 0;
        double dOffset = 0;
        int lastShooterPos = 0;
        double lastTime = getRuntime();
        double curVelocity = 0;
        boolean goingDown = false;
        robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.CAM.setPosition(0.57);
        waitForStart();
        robot.shooter.setPower(-shooterPow);
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        robot.magazine.setTargetPosition(0);
        robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.magazine.setPower(1);
        DcMotorEx shooter = (DcMotorEx)robot.shooter;
        while (opModeIsActive()) {
            FunctionLibrary.Point position = robot.getPosition();
            telemetry.addData("position", position.x + ", " + position.y);

            double x = FunctionLibrary.scaleInput(gamepad1.left_stick_x);
            double y = FunctionLibrary.scaleInput(gamepad1.left_stick_y);
            double dX;
            double dY;
            double rotation = FunctionLibrary.scaleInput(gamepad1.right_stick_x);
            telemetry.addData("fieldCentric", fieldCentric);
            if (!leftStickButton && gamepad1.left_bumper) {
                if (fieldCentric) {
                    fieldCentric = false;
                } else fieldCentric = true;
                dOffset = -GetYaw(0,robot.imu);
                leftStickButton = true;
            }
            else if (!gamepad1.left_bumper && leftStickButton) leftStickButton = false;
            if (!rightStickButton && gamepad1.right_bumper) {
                dOffset = -GetYaw(0,robot.imu);
                rightStickButton = true;
            }

            else if(!gamepad1.right_bumper && rightStickButton) rightStickButton = false;
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

            if (!isYPressed && gamepad1.a) {
                double intake = robot.intakeRD.getPower();
                double power = intake == 1 ? 0 : 1;
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                isYPressed = true;
            } else if(isYPressed && !gamepad1.a) {
                isYPressed = false;
            }
            if (!isAPressed && gamepad1.x) {
                double intake = robot.intakeRD.getPower();
                double power = intake == -1 ? 0 : -1;
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                isAPressed = true;
            } else if (isYPressed && !gamepad1.x) {
                isAPressed = false;
            }

            if (!xIsPressed && gamepad1.y) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos < 0 ? robot.magazine.getCurrentPosition()+magazineTurnRate : targetPos+magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                xIsPressed = true;
            } else if(xIsPressed && !gamepad1.y) {
                xIsPressed = false;
            }
            if (!bIsPressed && gamepad1.b) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos > 0 ? robot.magazine.getCurrentPosition()-magazineTurnRate : targetPos-magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bIsPressed = true;
            } else if (bIsPressed && !gamepad1.b) {
                bIsPressed = false;
            }


            if (!leftBumperIsPressed && gamepad1.left_trigger > 0) {
                double shooterPower = robot.shooter.getPower();
                robot.shooter.setPower(shooterPower == 0 || shooterPower == -shooterPow ? shooterPow : 0);
                leftBumperIsPressed = true;
            } else if(leftBumperIsPressed && !(gamepad1.left_trigger > 0)) {
                leftBumperIsPressed = false;
            }
            if (!rightBumperIsPressed && gamepad1.right_trigger > 0.1) {
                double shooterPower = robot.shooter.getPower();
                robot.shooter.setPower(shooterPower == 0 || shooterPower == shooterPow ? -shooterPow : 0);
                rightBumperIsPressed = true;
            } else if (rightBumperIsPressed && !(gamepad1.right_trigger > 0.1)) {
                rightBumperIsPressed = false;
            }
            if (gamepad1.right_stick_button && !dpadRightIsPressed) {

                if (!goingDown) wobblePivotPos = (wobblePivotPos+1)%3;
                else wobblePivotPos = wobblePivotPos == 0 ? 2 : wobblePivotPos-1;
                if (wobblePivotPos == 2) goingDown = true;
                else if (wobblePivotPos == 0) goingDown = false;
                dpadRightIsPressed = true;
            } else if (!gamepad1.right_stick_button){
                dpadRightIsPressed = false;
            }
            robot.wobblePivot.setPosition(wobblePivotPos == 0 ? 0 : wobblePivotPos == 1 ? 0.45 : 1);
            if (gamepad1.left_stick_button && !dpadLeftIsPressed) {
                robot.wobbleGrab.setPosition(robot.wobbleGrab.getPosition() == 0 ? 1 : 0);
                dpadLeftIsPressed = true;
            } else if (!gamepad1.left_stick_button){
                dpadLeftIsPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownIsPressed) {
                speedMultiplier = speedMultiplier == 0.25 ? 1 : 0.25;
                dpadDownIsPressed = true;
            } else if (!gamepad1.dpad_down && dpadDownIsPressed) {
                dpadDownIsPressed = false;
            }
            if (gamepad1.dpad_up && !dpadUpPressed) {
                CAMPos = (CAMPos+1)%2;
                robot.CAM.setPosition(camPoses[CAMPos]);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up && dpadUpPressed) {
                dpadUpPressed = false;
            }
            robot.CAM.setPosition(robot.CAM.getPosition()+((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0))*getRuntime());
            resetStartTime();
            robot.move(dX*speedMultiplier,dY*speedMultiplier,rotation*speedMultiplier,1);
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.addData("pivotPos", wobblePivotPos);
            double curCAMPos = Math.round(robot.CAM.getPosition()*100)/100;
            telemetry.addData("CamPos", curCAMPos);
            boolean camHadPos = false;
            for (int i = 0; i < camPoses.length; i++) {
                if (camPoses[i] == curCAMPos) {
                    telemetry.addData("CamPos", camPosNames[i]);
                    camHadPos = true;
                    break;
                }
            }
            if (!camHadPos) telemetry.addData("CamPos", curCAMPos);
            telemetry.addData("CAMPos", curCAMPos);
            telemetry.addData("Shooter", shooter.getVelocity());
            telemetry.addData("ShooterVol", shooter.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
    }
}
