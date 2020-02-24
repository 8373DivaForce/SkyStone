package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TwoSErvos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo a = hardwareMap.crservo.get("a");
        CRServo b = hardwareMap.crservo.get("b");
        waitForStart();
        while (opModeIsActive()) {
            a.setPower(gamepad1.left_stick_y);
            b.setPower(gamepad1.right_stick_y);
        }
    }
}
