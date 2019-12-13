package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

@TeleOp
public class AutoPicker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String[] alliances = {
            "red",
            "blue"
        };
        String[] Autos = {
                "Bridge",
                "Foundation",
                "Skystone"
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

        HashMap<String, Integer> autoValues = new HashMap<>();
        String[] thingsToControl = {
                "Alliance",
                "Auto",
                "Position",
                "EndPosition"
        };
        for (String s : thingsToControl) {
            autoValues.put(s,0);
        }

        boolean aPressed = false;
        boolean yPressed = false;
        boolean dUpPressed = false;
        boolean dDownPressed = false;
        while (opModeIsActive()) {
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
            if (gamepad1.dpad_up && !dUpPressed) {
                autoValues.put(valueSelected, (autoValues.get(valueSelected)+1)%(maxIteration));
            } else if (!gamepad1.dpad_up && dUpPressed) dUpPressed = false;

            if (gamepad1.dpad_down && !dDownPressed) {
                int valueToPut = (autoValues.get(valueSelected)-1)%(maxIteration);
                if (valueToPut < 0) valueToPut += maxIteration;
                autoValues.put(valueSelected, valueToPut);
                dDownPressed = true;
            } else if (!gamepad1.dpad_down && dDownPressed) dDownPressed = false;

            for (Map.Entry<String, Integer> entry : autoValues.entrySet()) {
                telemetry.addData(entry.getKey(), entry.getValue());
            }
            telemetry.update();
        }
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
