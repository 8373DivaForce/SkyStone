package org.firstinspires.ftc.teamcode.Libraries.functions;

//program for finding launch angle given x displacement, y displacement, and an initial velocity
public class ProjectileMotion {
    //"brute force method" iterates through every possible degree and checks to see which one works
    //has optimizations such that the first iteration can be up to 90 values, however, subsequent ones will be less than 20
    //after the first run, it iterates +-precision level from the closest angle
    //this means it will only look at values immediately surrounding the previous closest value to make it more precise
    public static double getTheta(double dX, double dY, double v) {
        double closest = 9999999;
        double closestAngle = 0;
        double minA = 0;
        double maxA = 90;
        double precision = 1;
        double maxError = 0.000000000000508;
        //get an initial estimation on the launch angle based on a straight line trajectory
        int aTheta = (int)Math.ceil(Math.toDegrees(Math.atan2(dY,dX)));
        //set the minimum to that value to reduce the number of calculations
        minA = aTheta;
        int wentBack = 0;
        double lastError = 9999999;
        int levels = 0;
        //iterate until it finds a close enough value
        while (true) {
            //iterate through every value within the minimum and maximum at a certain level of precision
            //It starts with steps of 1 degree and then gets smaller and smaller as it iterates more
            for (int i = 0; i < Math.floor((maxA - minA) / precision); i++) {
                //convert degrees to radians for our trig function
                double radians = Math.toRadians((i * precision) + minA);
                //get the time until it gets to the target using projectile motion
                double t = dX / (v * Math.cos(radians));
                //make sure the time isn't negative as it could impact our calculations
                if (t > 0) {
                    //check where it would be along the y axis at that point
                    double y = (v * Math.sin(radians) * t) + (-4.9 * (t*t));
                    //check if this is the closest value, if so, make this the closest one
                    if (Math.abs(y - dY) < Math.abs(closest - dY)) {
                        closest = y;
                        closestAngle = i * precision + minA;
                    }
                    //if we are getting further and further away from a correct answer, abandon this iteration
                    if (Math.abs(y - dY) > lastError) {
                        if (wentBack > 3) break;
                        else wentBack += 1;
                    } else {
                        wentBack = 0;
                    }
                    lastError = Math.abs(y - dY);
                }
            }
            //set the new mins and maxes in a range of the closest found angle
            minA = closestAngle - precision;
            maxA = closestAngle + precision;
            //make it 10x more precise
            precision = precision / 10;
            levels += 1;
            //if we are within an acceptable degree of accuracy, return that value and stop
            if (Math.abs(closest - dY) < maxError) break;
            //if we still haven't found a good value after 20 iterations, assume it is impossible
            if (levels > 20) return -9999;
        }
        //return the closest angle we were able to find
        return closestAngle;
    }
}

