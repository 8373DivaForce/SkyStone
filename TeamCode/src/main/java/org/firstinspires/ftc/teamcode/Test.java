package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left = hardwareMap.dcMotor.get("left");
        DcMotor right = hardwareMap.dcMotor.get("right");
        DcMotor lift = hardwareMap.dcMotor.get("wrist");
        DcMotor wrist = hardwareMap.dcMotor.get("lift");

        Servo claw = hardwareMap.servo.get("claw");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            left.setPower(gamepad1.left_stick_y);
            right.setPower(gamepad1.right_stick_y);

            if (gamepad1.right_bumper) {
                claw.setPosition(1);
            } else if (gamepad1.left_bumper) {
                claw.setPosition(-1);
            }


            lift.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

            if (gamepad1.dpad_up) {
                wrist.setPower(1);
            } else if (gamepad1.dpad_down) {
                wrist.setPower(-1);
            } else {
                wrist.setPower(0);
            }
        }
    }
}
