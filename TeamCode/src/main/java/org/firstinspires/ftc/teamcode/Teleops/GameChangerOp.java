package org.firstinspires.ftc.teamcode.Teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware_Maps.GameChangerBotHardware;
import org.firstinspires.ftc.teamcode.Libraries.Bases.RobotConstructor;
import org.firstinspires.ftc.teamcode.Libraries.Bases.task;
import org.firstinspires.ftc.teamcode.Libraries.functions.FunctionLibrary;
import org.firstinspires.ftc.teamcode.Libraries.functions.baseTasks;
import org.firstinspires.ftc.teamcode.Libraries.functions.taskHandler;
import org.firstinspires.ftc.teamcode.robotConstants;
import org.firstinspires.ftc.teamcode.worldVariables;

@TeleOp
@Config
public class GameChangerOp extends LinearOpMode {
    boolean wobblePivotUp = true;
    public static class waitForSpeedUp implements task {
        //initial variables for auto task
        private final double timeOut;
        private double startTime = 0;
        private int shooterSpeed = 120;
        public waitForSpeedUp(double timeOut) {
            this.timeOut = timeOut;
        }
        public waitForSpeedUp(double timeOut, int shooterSpeed) {
            this.timeOut = timeOut;
            this.shooterSpeed = shooterSpeed;
        }
        @Override
        public void init(RobotConstructor robot) {
            //set the start time for the timeout and set the motors to run using encoders
            startTime = System.currentTimeMillis();
        }

        @Override
        public int loop(RobotConstructor robot) {
            //check if the program has ran for too long, if so, terminate it
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;
            //iterate through each motor
            if (Math.abs(((DcMotorEx) GameChangerOp.robot.shooter).getVelocity()) > robotConstants.shooterSpeed-shooterSpeed) return -1;
            //return that the program is still running if it has gotten to this point
            return 1;
        }
    }
    public static class runTillSpeedDown implements task {
        //initial variables for auto task
        private final double timeOut;
        private double startTime = 0;
        private int shooterSpeed = 120;
        public runTillSpeedDown(double timeOut) {
            this.timeOut = timeOut;
        }
        public runTillSpeedDown(double timeOut, int shooterSpeed) {
            this.timeOut = timeOut;
            this.shooterSpeed = shooterSpeed;
        }
        @Override
        public void init(RobotConstructor robot) {
            //set the start time for the timeout and set the motors to run using encoders
            startTime = System.currentTimeMillis();
            GameChangerOp.robot.magazine.setPower(0);
            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        @Override
        public int loop(RobotConstructor robot) {
            GameChangerOp.robot.magazine.setPower(1);
            //check if the program has ran for too long, if so, terminate it
            if (System.currentTimeMillis()-startTime >= timeOut) return -2;
            //iterate through each motor
            if (Math.abs(((DcMotorEx) GameChangerOp.robot.shooter).getVelocity()) < robotConstants.shooterSpeed-shooterSpeed) {
                GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                GameChangerOp.robot.magazine.setTargetPosition(0);
                GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                GameChangerOp.robot.magazine.setPower(1);
                return -1;
            }
            //return that the program is still running if it has gotten to this point
            return 1;
        }
    }
    //constants for teleop run
    public static int magazineTurnRate = 280;
    public static double camPosHigh = 0.85;
    public static double camPosPow = 0.64;
    double speedMultiplier = 1;
    public static double shooterPow = 0.7;
    int CAMPos = 0;
    int shooterDir = 1;

    FunctionLibrary.Point startingPoint = new FunctionLibrary.Point(0,0);
    taskHandler handler = new taskHandler();
    boolean macroRunning = false;
    boolean secondMacroRunning = false;
    boolean doneWithTask = false;
    boolean moveOn = false;
    int curTask = 0;
    String[] camPosNames = {
            "High Goal",
            "Power Shot"
    };
    public static GameChangerBotHardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //initialization of variables like the robot and controller button states.
        robot = new GameChangerBotHardware(this,0,0, worldVariables.worldRotation);
        boolean aIsPressed = false;
        boolean dpadDownIsPressed = false;
        boolean xIsPressed = false;
        boolean fieldCentric = false;
        boolean leftBumperIsPressed = false;
        boolean rightBumperIsPressed = false;
        boolean yIsPressed = false;
        boolean bIsPressed = false;
        boolean leftTriggerIsPressed = false;
        boolean rightTriggerIsPressed = false;
        boolean leftStickIsPressed = false;
        boolean rightStickIsPressed = false;
        boolean dpadUpPressed = false;
        int wobblePivotPos = 0;
        double dOffset = 0;
        int lastShooterPos = 0;
        double lastTime = getRuntime();
        double curVelocity = 0;
        boolean goingDown = false;
        //setting up the magazine and shooter motors for operation
        robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //setting initial CAM position
        robot.CAM.setPosition(camPosHigh);
        //waits for the teleop to start
        waitForStart();
        //sets starting motor states
        robot.intakeRD.setPower(1);
        robot.deflector.setPower(1);
        robot.magazine.setTargetPosition(0);
        robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.magazine.setPower(1);
        DcMotorEx shooter = (DcMotorEx)robot.shooter;
        shooter.setVelocity(robotConstants.shooterSpeed);
        double lastShooterSpeed = robotConstants.shooterSpeed;
        task rotate = null;
        //main teleop loop
        while (opModeIsActive()) {
            if (lastShooterSpeed != robotConstants.shooterSpeed) {
                lastShooterSpeed = robotConstants.shooterSpeed;
                shooter.setVelocity(robotConstants.shooterSpeed);
            }
            telemetry.addData("Coefficients: ",shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            telemetry.addData("Magazine Coefficients: ",((DcMotorEx)robot.magazine).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            PIDFCoefficients pidfValsShooter = robotConstants.pidfValsShooter;
            PIDFCoefficients shooterPID = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (shooterPID.p != pidfValsShooter.p || shooterPID.i != pidfValsShooter.i || shooterPID.d != pidfValsShooter.d || shooterPID.f != pidfValsShooter.f) {
                shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfValsShooter);
            }
            double[] camPoses = {
                    camPosHigh,
                    camPosPow
            };
            telemetry.addData("Front1", robot.dcBackLeft.getCurrentPosition());
            telemetry.addData("Front2", robot.dcBackRight.getCurrentPosition());
            telemetry.addData("left", robot.dcFrontLeft.getCurrentPosition());
            //gets position for testing purposes
            FunctionLibrary.Point position = robot.getPosition();
            telemetry.addData("position", position.x + ", " + position.y);
            telemetry.addData("rotation", robot.getWorldRotation());
            //gets the controller x, y, and rotation movement from sticks and scales them
            double x = FunctionLibrary.scaleInput(gamepad1.left_stick_x);
            double y = FunctionLibrary.scaleInput(gamepad1.left_stick_y);
            double dX;
            double dY;
            double rotation = FunctionLibrary.scaleInput(gamepad1.right_stick_x);
            //returns whether or not fieldCentric is enabled.
            telemetry.addData("fieldCentric", fieldCentric);
            //checks leftBumper to toggle field centric on and off
            if (!leftBumperIsPressed && gamepad1.left_bumper && gamepad1.back) {
                rightBumperIsPressed = true;
                secondMacroRunning = false;
            }
            else if (!leftBumperIsPressed && gamepad1.left_bumper) {
                /*if (fieldCentric) {
                    fieldCentric = false;
                } else fieldCentric = true;
                dOffset = -robot.getWorldRotation();*/
                if (!secondMacroRunning && !macroRunning) {
                    handler = new taskHandler();
                    secondMacroRunning = true;
                    robot.CAM.setPosition(camPosHigh);
                    shooter.setVelocity(robotConstants.shooterSpeed);
                    shooterDir = 1;
                    handler.addTask(new waitForSpeedUp(1000,120));
                    handler.addTask(new runTillSpeedDown(1000,120));
                    handler.addTask(new waitForSpeedUp(1000,120));
                    handler.addTask(new runTillSpeedDown(1000,120));
                    handler.addTask(new waitForSpeedUp(1000,120));
                    handler.addTask(new runTillSpeedDown(1200,120));

                    handler.addTask(new task() {
                        @Override
                        public void init(RobotConstructor robot) {

                        }

                        @Override
                        public int loop(RobotConstructor robot) {
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            GameChangerOp.robot.magazine.setTargetPosition(0);
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            GameChangerOp.robot.magazine.setPower(1);
                            return -1;
                        }
                    });
                }
                secondMacroRunning = true;
                leftBumperIsPressed = true;
            }
            else if (!gamepad1.left_bumper && leftBumperIsPressed) leftBumperIsPressed = false;

            //checks the rightBumper to reset which direction is forwards for field centric
            if (!rightBumperIsPressed && gamepad1.dpad_up && gamepad1.back) {
                rightBumperIsPressed = true;
                macroRunning = false;
            }
            else if (!rightBumperIsPressed && gamepad1.dpad_up) {
                if (!macroRunning && !secondMacroRunning) {
                    handler = new taskHandler();
                    startingPoint = robot.getPosition();
                    double rot = robot.getWorldRotation();
                    macroRunning = true;
                    robot.CAM.setPosition(.43);
                    shooter.setVelocity(robotConstants.shooterSpeed);
                    shooterDir = 1;
                    robot.setRotation(0);
                    int magStartPos = robot.magazine.getCurrentPosition();
                    handler.addTask(new baseTasks.rotate(-4, 0.2, 1.5, 3000));
                    handler.addTask(new waitForSpeedUp(1000,80));
                    //handler.addTask(new baseTasks.motorMovement(robot.magazine,magStartPos+400,1,10,2000));
                    handler.addTask(new runTillSpeedDown(1000,80));
                    handler.addTask(new baseTasks.rotate(0, 0.2, 1.5, 3000));
                    handler.addTask(new waitForSpeedUp(1000,80));
                    //handler.addTask(new baseTasks.motorMovement(robot.magazine,magStartPos+900,1,10,2000));
                    handler.addTask(new runTillSpeedDown(1000,80));

                    handler.addTask(new baseTasks.rotate(8, 0.2, 1.5, 3000));
                    handler.addTask(new waitForSpeedUp(1000,80));
                    //handler.addTask(new baseTasks.motorMovement(robot.magazine,magStartPos+2500,1,10,2000));
                    handler.addTask(new runTillSpeedDown(1000,80));

                    handler.addTask(new task() {
                        @Override
                        public void init(RobotConstructor robot) {

                        }

                        @Override
                        public int loop(RobotConstructor robot) {
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            GameChangerOp.robot.magazine.setTargetPosition(0);
                            GameChangerOp.robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            GameChangerOp.robot.magazine.setPower(1);
                            return -1;
                        }
                    });
                }
                rightBumperIsPressed = true;
            }
            else if(!gamepad1.dpad_up && rightBumperIsPressed) rightBumperIsPressed = false;



            if (macroRunning || secondMacroRunning) {
                handler.loop(robot);
                if (handler.curTask == handler.numberOfTasks()) {
                    macroRunning = false;
                    secondMacroRunning = false;
                }
            }
            //translates the values for field centric if it is enabled
            if (fieldCentric) {
                //gets the vector angle for the controller inputs in degress
                double angle = Math.toDegrees(Math.atan2(y,x))+90;
                telemetry.addData("angle:", angle);
                //translates the vector inputs from the controller in relation to the robot angle
                angle = angle - robot.getWorldRotation() - dOffset;
                telemetry.addData("adjustedAngle", angle);
                //finds the hypot of that vector to use trig on
                double hyp = Math.sqrt((x*x) + (y*y));
                telemetry.addData("Rotation", robot.getWorldRotation());
                telemetry.addData("hyp", hyp);
                //finds the x and y movement from the translated vector
                dX = Math.cos(Math.toRadians(angle))*hyp * -1;
                dY = Math.sin(Math.toRadians(angle))*hyp;
            } else {
                dY = x;
                dX = y;
            }
            //enabled/disables the intake in the forwards direction when a is pressed
            if (!aIsPressed && gamepad1.a) {
                double intake = robot.intakeRD.getPower();
                double power = intake == 1 ? 0 : 1;
                //intake and deflector motors are linked
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                aIsPressed = true;
            } else if(aIsPressed && !gamepad1.a) {
                aIsPressed = false;
            }
            //enables/disables the intake in the backwards direction when x is pressed
            if (!xIsPressed && gamepad1.x) {
                double intake = robot.intakeRD.getPower();
                double power = intake == -1 ? 0 : -1;
                robot.intakeRD.setPower(power);
                robot.deflector.setPower(power);
                xIsPressed = true;
            } else if (xIsPressed && !gamepad1.x) {
                xIsPressed = false;
            }
            double shooterVelocity = shooter.getVelocity();
            //moves the magazine forwards the number of encoder ticks specified in the magazineTurnRate variable
            if (!yIsPressed && gamepad1.y && (Math.abs(shooterVelocity) > robotConstants.shooterSpeed-120 || gamepad1.back || shooterDir == 0 || shooterDir == -1)) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos < 0 ? robot.magazine.getCurrentPosition()+magazineTurnRate : targetPos+magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                yIsPressed = true;
            } else if(yIsPressed && !gamepad1.y) {
                yIsPressed = false;
            }
            //moves the magazine backwards the number of encoder ticks specified in the magazineTurnRate variable
            if (!bIsPressed && gamepad1.b) {
                int targetPos = robot.magazine.getTargetPosition();
                robot.magazine.setTargetPosition(targetPos > 0 ? robot.magazine.getCurrentPosition()-magazineTurnRate : targetPos-magazineTurnRate);
                robot.magazine.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bIsPressed = true;
            } else if (bIsPressed && !gamepad1.b) {
                bIsPressed = false;
            }

            //enables/disables the shooter in the backwards direction
            if (!leftTriggerIsPressed && gamepad1.left_trigger > 0.1) {
                if (shooterDir == 0 || shooterDir == 1) {
                    shooter.setVelocity(-robotConstants.shooterSpeed);
                    shooterDir = -1;
                } else {
                    shooter.setVelocity(0);
                    shooterDir = 0;
                }
                leftTriggerIsPressed = true;
            } else if(leftTriggerIsPressed && !(gamepad1.left_trigger > 0)) {
                leftTriggerIsPressed = false;
            }

            //enables/disables the shooter in the forwards direction
            if (!rightTriggerIsPressed && gamepad1.right_trigger > 0.1) {
                if (shooterDir == 0 || shooterDir == -1) {
                    shooter.setVelocity(robotConstants.shooterSpeed);
                    shooterDir = 1;
                } else {
                    shooter.setVelocity(0);
                    shooterDir = 0;
                }
                rightTriggerIsPressed = true;
            } else if (rightTriggerIsPressed && !(gamepad1.right_trigger > 0.1)) {
                rightTriggerIsPressed = false;
            }
            telemetry.addData("ShooterPower", robot.shooter.getPower());
            //iterates through the positions for the wobble goal mechanism
            if (gamepad1.right_stick_button && !rightStickIsPressed && gamepad1.back) {
                rightStickIsPressed = true;
                robot.wobblePivot.setPosition(0.9);
            } else if (gamepad1.right_stick_button && !rightStickIsPressed) {

                robot.wobblePivot.setPosition(wobblePivotUp ? 0.4 : 0);
                wobblePivotUp = !wobblePivotUp;
                rightStickIsPressed = true;
            } else if (!gamepad1.right_stick_button){
                rightStickIsPressed = false;
            }
            //opens/closes the grabber on the wobble goal mechanism when the left stick button is pressed
            if (gamepad1.left_stick_button && !leftStickIsPressed) {
                robot.wobbleGrab1.setPosition(robot.wobbleGrab1.getPosition() == 0 ? 1 : 0);
                robot.wobbleGrab2.setPosition(robot.wobbleGrab1.getPosition());
                leftStickIsPressed = true;
            } else if (!gamepad1.left_stick_button){
                leftStickIsPressed = false;
            }
            //enables/disables slow mode which halves the controller input for the drive motors to allow for finer control
            if (gamepad1.dpad_down && !dpadDownIsPressed) {
                speedMultiplier = speedMultiplier == 0.30 ? 1 : 0.30;
                dpadDownIsPressed = true;
            } else if (!gamepad1.dpad_down && dpadDownIsPressed) {
                dpadDownIsPressed = false;
            }
            //cycles through the preset CAM positions for the shooter
            if (gamepad1.right_bumper && !dpadUpPressed) {
                CAMPos = (CAMPos+1)%camPoses.length;
                robot.CAM.setPosition(camPoses[CAMPos]);
                dpadUpPressed = true;
            } else if (!gamepad1.right_bumper && dpadUpPressed) {
                dpadUpPressed = false;
            }
            //allows for manual control of the CAM angle using dpad_left and dpad_right
            robot.CAM.setPosition(robot.CAM.getPosition()+((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0))*getRuntime());
            resetStartTime();
            if (!macroRunning && !secondMacroRunning)
                robot.move(dX*speedMultiplier,dY*speedMultiplier,rotation*speedMultiplier,1);
            //logs current variable states
            telemetry.addData("x: ", robot.getX());
            telemetry.addData("y: ", robot.getY());
            telemetry.addData("pivotPos", wobblePivotPos);
            double curCAMPos = Math.round(robot.CAM.getPosition()*100)/100;
            telemetry.addData("CAMPos", robot.CAM.getPosition());
            telemetry.update();

        }
    }
}
