package org.firstinspires.ftc.teamcode.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissBotHArdware;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;

@TeleOp
@Disabled
public class NewKissOp extends LinearOpMode {

    boolean intake = false;
    boolean isYPressed = false;
    boolean isAPressed = false;
    @Override
    public void runOpMode() throws InterruptedException {
        NewKissBotHArdware robot = new NewKissBotHArdware(this,0,0,0);
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

            if (!isYPressed && gamepad1.y) {
                robot.intake.setPower((robot.intake.getPower()+1)%2);
                isYPressed = true;
            } else if(isYPressed && !gamepad1.y) {
                isYPressed = false;
            }
            if (!isAPressed && gamepad1.a) {
                robot.intake.setPower((robot.intake.getPower()-1)%-2);
                isAPressed = true;
            } else if (isYPressed && !gamepad1.a) {
                isAPressed = false;
            }
            robot.move(dX,dY,rotation,1);
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.update();

        }
    }
}
