package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.autoBase;

import java.io.File;

@Autonomous
public class AllAutos extends LinearOpMode {
    private autoBase auto = null;
    @Override
    public void runOpMode() throws InterruptedException {
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        //read the file we store the information on which autonomous we are running
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        if (file.exists()) {
            String[] fileContents = ReadWriteFile.readFile(file).split(",");
            Alliance = Integer.parseInt(fileContents[0]);
            Auto = Integer.parseInt(fileContents[1]);
            Position = Integer.parseInt(fileContents[2]);
            EndPosition = Integer.parseInt(fileContents[3]);
        } else {
            //if the file isn't there, stop the program
            stop();
        }
        if (Auto == 0) { //park
            auto = new ParkAutos(this);
        } else if (Auto == 1) { //wobble goal and park
            auto = new WobbleAndPark(this);
        } else {
            stop();
        }
        if (auto != null) {
            auto.init(Alliance, Auto, Position, EndPosition);
            while (!opModeIsActive() && !isStopRequested()) {
                auto.init_loop();
            }
            auto.loop_init();
            while (opModeIsActive()) {
                auto.loop();
            }
            auto.end();
        }
    }
}
