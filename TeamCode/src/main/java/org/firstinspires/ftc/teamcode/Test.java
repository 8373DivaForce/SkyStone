package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware_Maps.Kisshardware;
import org.firstinspires.ftc.teamcode.Hardware_Maps.NewKissHardware;

@TeleOp
public class Test extends LinearOpMode {
    public static void setRunMode(DcMotor[] motors, DcMotor.RunMode runMode) {
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor dcFrontLeft, dcFrontRight, dcBackLeft, dcBackRight;
        HardwareMap hMap = hardwareMap;
        dcFrontLeft = hMap.dcMotor.get("frontLeft");
        dcFrontRight = hMap.dcMotor.get("frontRight");
        dcBackLeft = hMap.dcMotor.get("backLeft");
        dcBackRight = hMap.dcMotor.get("backRight");

        DcMotor[] motors = {
                dcFrontLeft,
                dcFrontRight,
                dcBackLeft,
                dcBackRight
        };
        setRunMode(motors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setRunMode(motors, DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        int ticksPerRev = (int)dcFrontLeft.getMotorType().getTicksPerRev();
        while (opModeIsActive()) {
            dcFrontLeft.setTargetPosition(ticksPerRev);
            dcFrontRight.setTargetPosition(ticksPerRev);
            dcBackLeft.setTargetPosition(ticksPerRev);
            dcBackRight.setTargetPosition(ticksPerRev);

            dcFrontLeft.setPower(1);
            dcFrontRight.setPower(1);
            dcBackLeft.setPower(1);
            dcBackRight.setPower(1);
            setRunMode(motors, DcMotor.RunMode.RUN_TO_POSITION);

        }

    }
}
