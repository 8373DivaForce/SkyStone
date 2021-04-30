package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class robotConstants {
    public static PIDFCoefficients pidfValsShooter = new PIDFCoefficients(-50,0,-30,-11);
    public static double shooterSpeed = 1700;
}
