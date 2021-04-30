package org.firstinspires.ftc.teamcode.Autonomous.GameChanger;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Libraries.Bases.autoBase;

import java.io.File;


//"Parent" autonomous program for GameChangerSeason
@Autonomous
public class AllAutos extends LinearOpMode {
    private autoBase auto = null;
    @Override
    public void runOpMode() {
        //initializing configured values
        int Alliance = 0;
        int Auto = 0;
        int Position = 0;
        int EndPosition = 0;
        //read the file we store the information on which autonomous we are running
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        if (file.exists()) {
            String unParsedFile = ReadWriteFile.readFile(file);
            if (!unParsedFile.contains(",")) stop();
            String[] fileContents = unParsedFile.split(",");
            if (fileContents.length != 4) stop();
            Alliance = Integer.parseInt(fileContents[0]);
            Auto = Integer.parseInt(fileContents[1]);
            Position = Integer.parseInt(fileContents[2]);
            EndPosition = Integer.parseInt(fileContents[3]);
        } else {
            //if the file isn't there, stop the program
            stop();
        }
        //Initialize one of autoBase's child classes that has an actual autonomous program
        switch(Auto) {
            case 0:
                auto = new ParkAutos(this);
                break;
            case 1:
                auto = new WobbleAndPark(this);
                break;
            case 2:
                auto = new PowerShot(this);
                break;
            case 3:
                auto = new PowerWobble(this);
                break;
            case 4:
                auto = new PowerWobble2(this);
                break;
            case 5:
                auto = new HighWobble(this);
                break;
            case 6:
                auto = new HighWobblePlus(this);
                break;
            default:
                stop();
        }

        if (auto != null) {
            //run the autonomous programs init phase
            auto.init(Alliance, Auto, Position, EndPosition);
            //while the program has been initialized but not started, run the auto's init loop
            while (!opModeIsActive() && !isStopRequested()) {
                auto.init_loop();
            }
            //run the initialization to the auto's main loop
            auto.loop_init();
            //keep running the auto's main loop until program is terminated
            while (opModeIsActive()) {
                auto.loop();
            }
            //Run the auto's ending functions before exiting out of program
            auto.end();
        }
    }
}
