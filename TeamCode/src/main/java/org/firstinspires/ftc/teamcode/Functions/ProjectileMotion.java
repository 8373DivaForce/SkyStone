package org.firstinspires.ftc.teamcode.Functions;

public class ProjectileMotion {
    public static double getTheta(double dX, double dY, double v) {
        double closest = 9999999;
        double closestAngle = 0;
        double minA = 0;
        double maxA = 90;
        double precision = 1;
        double maxError = 0.000000000000508;
        int aTheta = (int)Math.ceil(Math.toDegrees(Math.atan2(dY,dX)));
        minA = aTheta;
        int wentBack = 0;
        double lastError = 9999999;
        int levels = 0;
        while (true) {
            for (int i = 0; i < Math.floor((maxA - minA) / precision); i++) {
                double radians = Math.toRadians((i * precision) + minA);
                double t = dX / (v * Math.cos(radians));
                if (t > 0) {
                    double y = (v * Math.sin(radians) * t) + (-4.9 * (t*t));
                    if (Math.abs(y - dY) < Math.abs(closest - dY)) {
                        closest = y;
                        closestAngle = i * precision + minA;
                    }
                    if (Math.abs(y - dY) > lastError) {
                        if (wentBack > 3) break;
                        else wentBack += 1;
                    } else {
                        wentBack = 0;
                    }
                    lastError = Math.abs(y - dY);
                }
            }
            minA = closestAngle - precision;
            maxA = closestAngle + precision;
            precision = precision / 10;
            levels += 1;
            if (Math.abs(closest - dY) < maxError) break;
            if (levels > 20) return -9999;
        }
        return closestAngle;
    }
}

