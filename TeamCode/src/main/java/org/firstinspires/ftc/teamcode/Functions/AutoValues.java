package org.firstinspires.ftc.teamcode.Functions;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class AutoValues {
    public String[] alliances = {
            "red",
            "blue"
    };
    public String[] Autos = {
            "Bridge",
            "Foundation",
            "Skystone"
    };
    public String[] FinalPositions = {
            "Against Wall",
            "Against Neutral Bridge"
    };
    public String[][][] startingPositions = new String[Autos.length][][];
    public HashMap<String, Integer> positions = new HashMap<>();
    public HashMap<String, Integer> endPositions = new HashMap<>();
    private final Telemetry telemetry;
    public AutoValues(Telemetry telemetry) {
        this.telemetry = telemetry;
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




        positions.put("Bridge", 4);
        endPositions.put("Bridge",2);
        positions.put("Foundation", 2);
        endPositions.put("Foundation",2);
        positions.put("Skystone", 2);
        endPositions.put("Skystone",2);
    }
    public void translateValues(int alliance, int auto, int position, int endposition) {
        telemetry.addData("Alliance", alliances[alliance]);
        telemetry.addData("Auto", Autos[auto]);
        telemetry.addData("Positoin", startingPositions[auto][alliance][position]);
        telemetry.addData("EndPosition", FinalPositions[endposition]);
    }
}
