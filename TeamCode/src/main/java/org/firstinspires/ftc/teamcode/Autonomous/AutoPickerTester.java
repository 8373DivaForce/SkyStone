package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp(group = "C")
public class AutoPickerTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //read the file we read from for autos and print them out for debugging/checking purposes
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        waitForStart();
        String writeFile = ReadWriteFile.readFile(file);
        while (opModeIsActive()) {
            telemetry.addData("FileContents", writeFile);
            telemetry.update();
        }
    }
}
