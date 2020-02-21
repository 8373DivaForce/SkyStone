package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

@TeleOp(group="B")
public class AutoPicker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set up arrays to make it easier to convert int values to readable data
        String[] alliances = {
            "red",
            "blue"
        };
        String[] Autos = {
                "Bridge",
                "Foundation",
                "Skystone"
        };
        String[][][] startingPositions = new String[Autos.length][][];


        startingPositions[0] = new String[2][];
        startingPositions[0][0] = new String[4];
        startingPositions[0][1] = new String[4];
        startingPositions[0][0][0] = "Next To Quarry";
        startingPositions[0][0][1] = "Left of SkyBridge";
        startingPositions[0][0][2] = "Right of SkyBridge";
        startingPositions[0][0][3] = "Next to Build Site";

        startingPositions[0][1][0] = "Next To Quarry";
        startingPositions[0][1][1] = "Right of SkyBridge";
        startingPositions[0][1][2] = "Left of SkyBridge";
        startingPositions[0][1][3] = "Next to Build Site";


        startingPositions[1] = new String[2][];
        startingPositions[1][0] = new String[2];
        startingPositions[1][1] = new String[2];

        startingPositions[1][0][0] = "Right of SkyBridge";
        startingPositions[1][0][1] = "Next to Build site";

        startingPositions[1][1][0] = "Left of SkyBridge";
        startingPositions[1][1][1] = "Next to Build site";

        startingPositions[2] = new String[2][];
        startingPositions[2][0] = new String[2];
        startingPositions[2][1] = new String[2];

        startingPositions[2][0][0] = "Next to Quarry";
        startingPositions[2][0][1] = "Left of SkyBridge";

        startingPositions[2][1][0] = "Next to Quarry";
        startingPositions[2][1][1] = "Right of SkyBridge";

        String[] FinalPositions = {
                "Against Wall",
                "Against Neutral Bridge"
        };
        HashMap<String, Integer> positions = new HashMap<>();
        HashMap<String, Integer> endPositions = new HashMap<>();

        positions.put("Bridge", 4);
        endPositions.put("Bridge",2);
        positions.put("Foundation", 2);
        endPositions.put("Foundation",2);
        positions.put("Skystone", 2);
        endPositions.put("Skystone",2);
        waitForStart();

        int selectedControl = 0;

        //setup the things we can control and a hashmap for our values
        HashMap<String, Integer> autoValues = new HashMap<>();
        String[] thingsToControl = {
                "Alliance",
                "Auto",
                "Position",
                "EndPosition"
        };
        //default all values to 0
        for (String s : thingsToControl) {
            autoValues.put(s,0);
        }

        //setup variables for press once checks
        boolean aPressed = false;
        boolean yPressed = false;
        boolean dUpPressed = false;
        boolean dDownPressed = false;
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
                maxIteration = alliances.length;
            } else if (valueSelected == "Auto") {
                maxIteration = Autos.length;
            } else if (valueSelected == "Position") {
                maxIteration = positions.get(Autos[autoValues.get("Auto")]);
            } else if (valueSelected == "EndPosition") {
                maxIteration = endPositions.get(Autos[autoValues.get("Auto")]);
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
            telemetry.addData("Alliance", alliances[autoValues.get("Alliance")]);
            telemetry.addData("Auto", Autos[autoValues.get("Auto")]);
            telemetry.addData("Starting Position", startingPositions[autoValues.get("Auto")][autoValues.get("Alliance")][autoValues.get("Position")]);
            telemetry.addData("End Position", FinalPositions[autoValues.get("EndPosition")]);
            telemetry.update();
        }
        //save them to a file when finished
        File file = AppUtil.getInstance().getSettingsFile("AutoSelection");
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
