package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.Bases.task;
import org.firstinspires.ftc.teamcode.Libraries.functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.Point;

import org.firstinspires.ftc.teamcode.Hardware_Maps.OldKissBotHArdware;

import static org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary.GetYaw;

@Autonomous(group = "Calibration")
public class OdometryCalibrator extends LinearOpMode {
    public class move implements task {
        //initial variables needed for program
        double startTime = 0;
        final FunctionLibrary.Point targetPos;
        Double targetAngle = null;
        final double power;
        final double maxError;
        final double timeOut;
        //initialization function for getting where it needs to go, it's angle, it's power, maxError, and how long it should take
        public move(FunctionLibrary.Point pos, double angle, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.targetAngle = angle;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        //same as above except without a target angle to keep the robot at
        public move(FunctionLibrary.Point pos, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        //set the robot's start time on init so we can have the program terminate after a set period of time
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //get the robots current position
            FunctionLibrary.Point pos = robot.getPosition();
            //if this task has taken to long, stop the robot and terminate the program
            if (System.currentTimeMillis()-startTime >= timeOut) {
                robot.move(0,0,0,0);
                return -2;
                //if the robot has reached it's target destination, stop the robots movement and terminate the program
            } else if (Math.sqrt(Math.pow(pos.x-targetPos.x,2) + Math.pow(pos.y-targetPos.y,2)) <= maxError) {
                robot.move(0,0,0,0);
                return -1;
            }
            //get it's offset from where it wants to be
            FunctionLibrary.Point offset = new FunctionLibrary.Point(targetPos.x-pos.x,targetPos.y-pos.y);
            double distance = Math.sqrt((offset.x*offset.x) + (offset.y*offset.y));
            double rotation = GetYaw(0,robot.imu);
            if (rotation > 360) rotation %= 360;
            if (rotation < -360) rotation %= -360;
            if (rotation > 180) rotation -= 360;
            if (rotation < -180) rotation += 360;

            double currentRotation = rotation;
            //find the current angle
            double currentAngle = currentRotation;
            //find the movement vector angle
            double angle = Math.toDegrees(Math.atan2(offset.y,offset.x));
            //translate the local movement vector to a global vecctor
            double robotAngle = angle + currentAngle;

            //find the x movement using distance times the cosine of the angle calculated
            double adjustedX = distance*Math.cos(Math.toRadians(robotAngle));
            //find the y movement using distance time the sin of the angle calculated
            double adjustedY = -distance*Math.sin(Math.toRadians(robotAngle));

            //if it has an angle to go to, calculate how it needs to rotate, otherwise skip it
            if (targetAngle != null) {
                //find the current angle on a 0 to 360 degree span
                double currentAngle360 = currentAngle;
                if (currentAngle < 0) currentAngle360 = currentAngle+360;

                double targetAngle360 = targetAngle;
                if (targetAngle < 0) targetAngle360 = targetAngle+360;

                double robotRotation = 0;

                //check to see which way is faster, moving left or right and tell it to move accordingly
                if (Math.abs(targetAngle-currentAngle) < Math.abs(targetAngle360-currentAngle360)) {
                    robotRotation = targetAngle-currentAngle;
                } else {
                    robotRotation = targetAngle360-currentAngle360;
                }
                //pass the x movement, y movement, rotation, and power to the move function in the constructor class
                robot.move(adjustedY, adjustedX, robotRotation/50, power);
            } else {
                //if we don't have a target rotation, just give it it's x and y movement
                robot.move(adjustedY, adjustedX, 0,power);
            }
            //return that the program is still running
            return 1;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GameChangerBotHardware robot = new GameChangerBotHardware(this, 0,0,0);
        AutoFunctions auto = new AutoFunctions(robot);
        String choice = "";
        boolean deciding = true;
        //let the user specify a direction to move
        while (!isStarted() && !isStopRequested() && deciding) {
            if (gamepad1.dpad_up) choice = "Forward";
            else if (gamepad1.dpad_down) choice = "Backwards";
            else if (gamepad1.dpad_left) choice = "Left";
            else if (gamepad1.dpad_right) choice = "Right";
            telemetry.addData("choice", choice);
            telemetry.update();
            if (choice != "") deciding = false;
        }
        waitForStart();
        FunctionLibrary.Point destination = null;
        //based off of the choice, tell it to move 2 feet forwards, backwards, left, or right
        move moveTask;
        double distance = 100;
        if (choice == "Forward") moveTask = new move(new Point(0, distance), 0, 0.5, 1, 20000);
        else if (choice == "Backwards") moveTask = new move(new Point(0, -distance), 0, 0.5, 1, 20000);
        else if (choice == "Left") moveTask = new move(new Point(-distance, 0), 0, 0.5, 1, 20000);
        else if (choice == "Right") moveTask = new move(new Point(distance, 0), 0, 0.5, 1, 20000);
        else  moveTask = new move(new Point(0, 0), 0, 0.5, 1, 20000);

        int nSwitch = 0;
        double result = 0;
        while (opModeIsActive()) {
            telemetry.addData("Front1", robot.dcBackLeft.getCurrentPosition());
            telemetry.addData("Front2", (robot.dcBackRight.getCurrentPosition()*(0.8978179)) + "");
            telemetry.addData("left", robot.dcFrontLeft.getCurrentPosition());
            switch (nSwitch) {
                case 0:
                    moveTask.init();
                    nSwitch++;
                    break;
                case 1:
                    //tell the robot to move in the pecified direction
                    if (gamepad1.a)
                        result = moveTask.loop(robot);
                    else robot.move(0,0,0,0);
                    if (result < 0) nSwitch++;
                    break;
                case 2:
                    //rotate the robot back to it's original rotation
                    result = auto.rotPID(0, 1, 1, 5);
                    if (result < 0) nSwitch++;
                    break;
            }
            //print it's position and rotation
            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.addData("rotation", robot.getWorldRotation());
            telemetry.addData("State", nSwitch);
            telemetry.update();
        }
    }
}
