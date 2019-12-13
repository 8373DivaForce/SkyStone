package org.firstinspires.ftc.teamcode.Calibrations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Map;


//This class iterates through motors, CRServos, and servos and allows for their individual control
@TeleOp
public class CheckAllHardwareDevices extends LinearOpMode {
    //boolean that allows you to control two motors with the same joystick
    boolean slaveMotors = false;
    //power multiplier to allow you to reverse one of the motors
    double slavePowerMultiplier = 1;
    //Array containing the devices it can control
    String[] deviceTypes = {
            "Motors",
            "CRServos",
            "Servos"
    };
    //variables stored to allow the iteration and selection of devies
    int selectedDeviceType = 0;
    int selectedMotor = 0;
    int selectedServo = 0;
    int selectedCRServo = 0;
    int selectedMotor2 = 0;
    int selectedServo2 = 0;
    int selectedCRServo2 = 0;
    int numberOfMotors;
    int numberOfServos;
    int numberOfCRServos;
    //create an arraylist of entries for motors, servos, and CRServos
    ArrayList<Map.Entry<String, DcMotor>> motors = new ArrayList<>();
    ArrayList<Map.Entry<String, Servo>> servos = new ArrayList<>();
    ArrayList<Map.Entry<String, CRServo>> crServos = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        //define the amount of motors found for easy iteration
        numberOfMotors = hardwareMap.dcMotor.entrySet().size();
        //iterate through each entry in the dcMotor hashmap and store it in the arrylist
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            motors.add(entry);
        }
        //define the amount of servos for easy iteration
        numberOfServos = hardwareMap.servo.entrySet().size();
        //iterate through each entry in the servo hashmap and store it in the arraylist
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            servos.add(entry);
        }
        //define the amount of CRServos for easy iteration
        numberOfCRServos = hardwareMap.crservo.entrySet().size();
        //iterate through each entry in the CRServo hashmap and sotre it in the arraylist
        for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
            crServos.add(entry);
        }
        //boolean to allow the back button to toggle the slave motor
        boolean backNotPressed = true;
        waitForStart();
        while (opModeIsActive()) {
            //print out the selected device type
            telemetry.addData("Selected Device Type", deviceTypes[selectedDeviceType]);
            //save the current selected device type to make it easier to use
            String deviceType = deviceTypes[selectedDeviceType];

            //if statemtns to control allow iteration of the devices
            int leftSideModifier = 0;
            int rightSideModifier = 0;
            if (gamepad1.dpad_up) leftSideModifier = 1;
            if (gamepad1.dpad_down) leftSideModifier = -1;
            if (gamepad1.y) rightSideModifier = 1;
            if (gamepad1.a) rightSideModifier = -1;
            if (backNotPressed && gamepad1.back) {
                backNotPressed = false;
                slaveMotors = !slaveMotors;
            } else if(!backNotPressed && !gamepad1.back) {
                backNotPressed = true;
            }
            if (gamepad1.right_trigger > 0) slavePowerMultiplier = 1;
            if (gamepad1.left_trigger > 0) slavePowerMultiplier = -1;
            if (getRuntime() > 0.5) {
                if (gamepad1.right_bumper) {
                    selectedDeviceType++;
                    selectedDeviceType = selectedDeviceType%(deviceTypes.length);
                    resetStartTime();
                } else if (gamepad1.left_bumper) {
                    selectedDeviceType--;
                    if (selectedDeviceType < 0) selectedDeviceType = deviceTypes.length-1;
                    selectedDeviceType = selectedDeviceType%(deviceTypes.length);
                    resetStartTime();
                }
                if (gamepad1.dpad_up || gamepad1.dpad_down) {
                    if (deviceType == "Motors" && motors.size() > 1) {
                        selectedMotor += leftSideModifier;
                        if (selectedMotor < 0) selectedMotor = motors.size()-1;
                        selectedMotor = selectedMotor%(motors.size());
                        resetStartTime();
                    } else if (deviceType == "Servos" && servos.size() > 1) {
                        selectedServo += leftSideModifier;
                        if (selectedServo < 0) selectedServo = servos.size()-1;
                        selectedServo = selectedServo%(servos.size());
                        resetStartTime();
                    } else if (deviceType == "CRServos" && servos.size() > 1) {
                        selectedCRServo += leftSideModifier;
                        if (selectedCRServo < 0) selectedCRServo = servos.size()-1;
                        selectedCRServo = selectedCRServo%(crServos.size());
                        resetStartTime();
                    }
                }
                if (gamepad1.y || gamepad1.a) {
                    if (deviceType == "Motors" && motors.size() > 1) {
                        selectedMotor2 += rightSideModifier;
                        if (selectedMotor2 < 0) selectedMotor2 = motors.size()-1;
                        selectedMotor2 = selectedMotor2%(motors.size());
                        resetStartTime();
                    } else if (deviceType == "Servos" && servos.size() > 1) {
                        selectedServo2 += rightSideModifier;
                        if (selectedServo2 < 0) selectedServo2 = servos.size()-1;
                        selectedServo2 = selectedServo2%(servos.size());
                        resetStartTime();
                    } else if (deviceType == "CRServos" && servos.size() > 1) {
                        selectedCRServo2 += rightSideModifier;
                        if (selectedCRServo2 < 0) selectedCRServo2 = servos.size()-1;
                        selectedCRServo2 = selectedCRServo2%(crServos.size());
                        resetStartTime();
                    }
                }
            }

            //check the device type and based off of that allow control of the selected devices
            if (deviceType == "Motors" && motors.size() > 0) {
                telemetry.addData("slave motors: ", slaveMotors);
                telemetry.addData("multiplier: ", slavePowerMultiplier);
                if (slaveMotors) {
                    telemetry.addData("Motor 1", motors.get(selectedMotor).getKey());
                    motors.get(selectedMotor).getValue().setPower(gamepad1.left_stick_y);
                    telemetry.addData("Motor 2", motors.get(selectedMotor2).getKey());
                    motors.get(selectedMotor2).getValue().setPower(gamepad1.left_stick_y *slavePowerMultiplier);
                } else {
                    telemetry.addData("Motor 1", motors.get(selectedMotor).getKey());
                    motors.get(selectedMotor).getValue().setPower(gamepad1.left_stick_y);
                    telemetry.addData("Motor 2", motors.get(selectedMotor2).getKey());
                    motors.get(selectedMotor2).getValue().setPower(gamepad1.right_stick_y);
                }

            }
            if (deviceType == "Servos" && servos.size() > 0) {
                telemetry.addData("Servo 1", servos.get(selectedServo).getKey());
                Servo servo = servos.get(selectedServo).getValue();
                servo.setPosition(servo.getPosition()+(gamepad1.left_stick_y));
                telemetry.addData("Servo 2", servos.get(selectedServo2).getKey());
                Servo servo2 = servos.get(selectedServo2).getValue();
                servo2.setPosition(servo2.getPosition()+(gamepad1.right_stick_y));
            }
            if (deviceType == "CRServos" && crServos.size() > 0) {
                telemetry.addData("CRServo 1", crServos.get(selectedCRServo).getKey());
                crServos.get(selectedCRServo).getValue().setPower(gamepad1.left_stick_y);
                telemetry.addData("CRServo 2", crServos.get(selectedCRServo).getKey());
                crServos.get(selectedCRServo).getValue().setPower(gamepad1.right_stick_y);
            }

            //iterate through motors, servos, and crservos to make sure they get reset if they aren't
            //selected
            for (Map.Entry<String, DcMotor> entry : motors) {
                telemetry.addData(entry.getKey(), entry.getValue().getCurrentPosition());
                entry.getValue().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if ((entry != motors.get(selectedMotor) && entry != motors.get(selectedMotor2)) || deviceType != "Motors") entry.getValue().setPower(0);
            }
            for (Map.Entry<String, Servo> entry : servos) {
                telemetry.addData(entry.getKey(), entry.getValue().getPosition());
            }
            for (Map.Entry<String, CRServo> entry : crServos) {
                telemetry.addData(entry.getKey(), entry.getValue().getPower());
                if ((entry != crServos.get(selectedCRServo) && entry != crServos.get(selectedCRServo2)) || deviceType != "CRServos") entry.getValue().setPower(0);
            }

            telemetry.update();
        }
    }
}
