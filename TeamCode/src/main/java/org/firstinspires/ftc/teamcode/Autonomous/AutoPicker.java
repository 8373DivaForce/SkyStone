package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Functions.AutoValues;

import java.io.File;
import java.util.HashMap;

@TeleOp(group="B")
public class AutoPicker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        AutoValues stringValues = new AutoValues(telemetry);
        //set up arrays to make it easier to convert int values to readable data


        //setup the things we can control and a hashmap for our values
        HashMap<String, Integer> autoValues = new HashMap<>();
        String[] thingsToControl = {
                "Alliance",
                "Auto",
                "Position",
                "EndPosition"
        };
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        if (file.exists()) {
            String[] fileContents = ReadWriteFile.readFile(file).split(",");
            for (int i = 0; i < thingsToControl.length; i++) {
                autoValues.put(thingsToControl[i], Integer.parseInt(fileContents[i]));
            }
        } else {
            for (String s : thingsToControl) {
                autoValues.put(s,0);
            }
        }
        int selectedControl = 0;
        //setup variables for press once checks
        boolean aPressed = false;
        boolean yPressed = false;
        boolean dUpPressed = false;
        boolean dDownPressed = false;
        waitForStart();
        while (opModeIsActive()) {
            //iterate through values based off of what buttons are being pressed
            if (gamepad1.a && !aPressed) {

                selectedControl = (selectedControl+1)%(thingsToControl.length);

                aPressed = true;
            }
            else if (!gamepad1.a && aPressed) aPressed = false;

            if (gamepad1.y && !yPressed) {
                selectedControl = (selectedControl-1)%(thingsToControl.length);
                if (selectedControl < 0) selectedControl += thingsToControl.length;
                yPressed = true;
            } else if (!gamepad1.y && yPressed) yPressed = false;

            String valueSelected = thingsToControl[selectedControl];
            telemetry.addData("Selected Value: ", valueSelected);
            int maxIteration = 0;
            //set the max value we can iterate to based off of what we are controlling
            if (valueSelected == "Alliance") {
                maxIteration = stringValues.alliances.length;
            } else if (valueSelected == "Auto") {
                maxIteration = stringValues.Autos.length;
            } else if (valueSelected == "Position") {
                maxIteration = stringValues.positions.get(stringValues.Autos[autoValues.get("Auto")]);
            } else if (valueSelected == "EndPosition") {
                maxIteration = stringValues.endPositions.get(stringValues.Autos[autoValues.get("Auto")]);
            }
            telemetry.addData("maxIteration", maxIteration);
            //allow iterating position, endposition, auto, and alliance
            if (gamepad1.dpad_up && !dUpPressed) {
                autoValues.put(valueSelected, (autoValues.get(valueSelected)+1)%(maxIteration));
            } else if (!gamepad1.dpad_up && dUpPressed) dUpPressed = false;

            if (gamepad1.dpad_down && !dDownPressed) {
                int valueToPut = (autoValues.get(valueSelected)-1)%(maxIteration);
                if (valueToPut < 0) valueToPut += maxIteration;
                autoValues.put(valueSelected, valueToPut);
                dDownPressed = true;
            } else if (!gamepad1.dpad_down && dDownPressed) dDownPressed = false;

            //print out the current values for everything
            stringValues.translateValues(autoValues.get("Alliance"), autoValues.get("Auto"), autoValues.get("Position"), autoValues.get("EndPosition"));
            telemetry.update();
        }
        //save them to a file when finished
        file = AppUtil.getInstance().getSettingsFile("AutoSelection");
        String writefile = "";
        for (int i = 0; i < thingsToControl.length; i++) {
            writefile += autoValues.get(thingsToControl[i]);
            if (i+1 != thingsToControl.length) {
                writefile += ",";
            }
        }
        ReadWriteFile.writeFile(file, writefile);
    }
}
