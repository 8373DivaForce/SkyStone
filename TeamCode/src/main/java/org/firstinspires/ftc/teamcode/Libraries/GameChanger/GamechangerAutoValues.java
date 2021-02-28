package org.firstinspires.ftc.teamcode.Libraries.GameChanger;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GamechangerAutoValues {
    //setup initial variables
    public String[] alliances = {
            "blue",
            "red"
    };
    public String[] Autos = {
            "Park",
            "Wobble goal",
            "Power Shot",
            "Power and Wobble"
    };
    public String[] FinalPositions = {
            "Right",
            "Left"
    };
    public String[] startingPositions = {
            "Right",
            "Left"
    };
    private final Telemetry telemetry;
    public GamechangerAutoValues(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //take in the values that were fed in and output them in text form through telemetry
    public void translateValues(int alliance, int auto, int position, int endposition) {
        telemetry.addData("Alliance", alliances[alliance]);
        telemetry.addData("Auto", Autos[auto]);
        telemetry.addData("Position", startingPositions[position]);
        telemetry.addData("EndPosition", FinalPositions[endposition]);
    }
}
