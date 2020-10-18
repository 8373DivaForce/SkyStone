package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.math.BigInteger;

public class baseTasks {

    public static class runUntil implements task {
        private DcMotor[] motors = null;
        private CRServo[] CRServos = null;
        private final TouchSensor[] sensors;
        private final double[] powers;


        public runUntil(DcMotor[] motors, double[] powers, TouchSensor[] sensors) {
            this.motors = motors;
            this.powers = powers;
            this.sensors = sensors;
        }
        public runUntil(DcMotor motor, double power, TouchSensor[] sensors) {
            this(new DcMotor[]{motor}, new double[]{power},sensors);
        }
        public runUntil(DcMotor[] motors, double power, TouchSensor[] sensors) {
            this.motors = motors;
            powers = new double[motors.length];
            for (int i = 0; i < motors.length; i++) {
                powers[i] = power;
            }
            this.sensors = sensors;
        }
        public runUntil(CRServo[] CRServos, double[] powers, TouchSensor[] sensors) {
            this.CRServos = CRServos;
            this.powers = powers;
            this.sensors = sensors;
        }
        public runUntil(CRServo servo, double power, TouchSensor[] sensors) {
            this(new CRServo[]{servo}, new double[]{power}, sensors);
        }
        public runUntil(CRServo[] CRServos, double power, TouchSensor[] sensors) {
            this.CRServos = CRServos;
            powers = new double[CRServos.length];
            for (int i = 0; i < CRServos.length; i++) {
                powers[i] = power;
            }
            this.sensors = sensors;
        }
        @Override
        public void init() {
            if (motors != null) {
                for (DcMotor motor : motors) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }
        }

        @Override
        public int loop(RobotConstructor robot) {
            for (TouchSensor sensor : sensors) {
                if (sensor.isPressed()) return -1;
            }
            if (motors != null) {
                if (motors.length != powers.length) return -3;
                for (int i = 1; i < motors.length; i++) {
                    motors[i].setPower(powers[i]);
                }
            } else if(CRServos != null) {
                if (CRServos.length != powers.length) return -3;
                for (int i = 1; i < CRServos.length; i++) {
                    CRServos[i].setPower(powers[i]);
                }
            } else return -2;
            return 1;
        }
    }
    public static class move implements task {
        double startTime = 0;
        final FunctionLibrary.Point targetPos;
        Double targetAngle = null;
        final double power;
        final double maxError;
        final double timeOut;
        public move(FunctionLibrary.Point pos, double angle, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.targetAngle = angle;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        public move(FunctionLibrary.Point pos, double power, double maxError, double timeOut) {
            this.targetPos = pos;
            this.power = power;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            FunctionLibrary.Point pos = robot.getPosition();
            if (System.currentTimeMillis()-startTime >= timeOut) {
                robot.move(0,0,0,0);
                return -2;
            } else if (Math.sqrt(Math.pow(pos.x-targetPos.x,2) + Math.pow(pos.y-targetPos.y,2)) <= maxError) {
                robot.move(0,0,0,0);
                return -1;
            }
            FunctionLibrary.Point offset = new FunctionLibrary.Point(targetPos.x-pos.x,targetPos.y-pos.y);
            double distance = Math.sqrt((offset.x*offset.x) + (offset.y*offset.y));
            double currentRotation = robot.getWorldRotation();
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
                robot.move(adjustedY, adjustedX, 0,power);
            }
            return 1;
        }
    }
    public static class motorMovement implements task{
        private final DcMotor[] motors;
        private final int[] positions;
        private final double maxError;
        private final double maxPower;
        private final double timeOut;
        private double startTime = 0;

        public motorMovement(DcMotor[] motors, int[] positions, double maxPower, double maxError, double timeOut) {
            this.maxPower = maxPower;
            this.maxError = maxError;
            this.timeOut = timeOut;
            this.motors = motors;
            this.positions = positions;
        }
        public motorMovement(DcMotor motor, int position, double maxPower, double maxError, double timeOut) {
            this.motors = new DcMotor[]{motor};
            this.positions = new int[]{position};
            this.maxPower = maxPower;
            this.maxError = maxError;
            this.timeOut = timeOut;
        }
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        @Override
        public int loop(RobotConstructor robot) {
            double error = 0;
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;
            if (motors.length != positions.length) return -1;
            for (int i = 0; i < motors.length; i++) {
                error += Math.abs(motors[i].getCurrentPosition()-positions[i]);
                motors[i].setPower(maxPower);
                motors[i].setTargetPosition(positions[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            error /= motors.length;
            if (error <= maxError) return -1;
            return 1;
        }
    }
    public static class servoMovement implements task {
        private final Servo[] servos;
        private final double[] positions;
        private final double time;
        public servoMovement(Servo[] servos, double[] positions, double time) {
            this.servos = servos;
            this.positions = positions;
            this.time = time;
        }
        public servoMovement(Servo servo, double position, double time) {
            this.servos = new Servo[]{servo};
            this.positions = new double[]{position};
            this.time = time;
        }
        private double startTime = 0;
        @Override
        public void init() {
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            if (servos.length != positions.length) return -3;
            for (int i = 0; i < servos.length; i++) {
                servos[i].setPosition(positions[i]);
            }
            if (System.currentTimeMillis()-startTime <= time) {
                return -1;
            }
            return 1;
        }
    }
}
