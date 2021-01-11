package org.firstinspires.ftc.teamcode.Libraries.functions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class controllerHandler {
    //default cooldown time for all the buttons
    private double cooldownTime = 0.5;
    //initialization that allows for the user to set that default time
    public controllerHandler(double cooldownTime) {
        this.cooldownTime = cooldownTime;
    }
    public controllerHandler() { }
    //Defining variable for keeping the time
    ElapsedTime time = new ElapsedTime();

    //defining all the variables needed for each button
    //Pressed: true if the button is currently being pressed
    //FirstPressed: returns true if this is the first iteration in which the button was pressed
    //Held: returns true on first press and at a certain interval while it's still being pressed
    //Toggle: flips state every time FirstPressed is true
    //Time: the last time the button was pressed or triggered a variable
    //Cooldown: User modifiable variable for adjusting the cooldown used for the held variable

    public boolean leftStickPressed, rightStickPressed = false;
    public boolean leftStickFirstPress, rightStickFirstPress = false;
    public boolean leftStickHeld, rightStickHeld = false;
    public boolean rightStickToggle, leftStickToggle = false;
    private double leftStickTime, rightStickTime = 0;
    public double leftStickCooldown, rightStickCooldown = cooldownTime;

    public boolean dpadLeftPressed, dpadRightPressed, dpadDownPressed, dpadUpPressed = false;
    public boolean dpadLeftFirstPress, dpadRightFirstPress, dpadDownFirstPress, dpadUpFirstPress = false;
    public boolean dpadLeftHeld, dpadRightHeld, dpadDownHeld, dpadUpHeld = false;
    public boolean dpadLeftToggle, dpadRightToggle, dpadDownToggle, dpadUpToggle = false;
    private double dpadLeftTime, dpadRightTime, dpadDownTime, dpadUpTime = 0;
    public double dpadLeftCooldown, dpadRightCooldown, dpadDownCooldown, dpadUpCooldown = cooldownTime;

    public boolean aPressed, bPressed, yPressed, xPressed = false;
    public boolean aFirstPress, bFirstPress, yFirstPress, xFirstPress = false;
    public boolean aHeld, bHeld, yHeld, xHeld = false;
    public boolean aToggle, bToggle, yToggle, xToggle = false;
    private double aTime, bTime, yTime, xTime = 0;
    public double aCooldown, bCooldown, yCooldown, xCooldown = cooldownTime;

    public boolean leftBumperPressed, rightBumperPressed = false;
    public boolean leftBumperFirstPress, rightBumperFirstPress = false;
    public boolean leftBumperHeld, rightBumperHeld = false;
    public boolean leftBumperToggle, rightBumperToggle = false;
    private double leftBumperTime, rightBumperTime = 0;
    public double leftBumperCooldown, rightBumperCooldown = cooldownTime;

    public boolean startButtonPressed, backButtonPressed = false;
    public boolean startButtonFirstPress, backButtonFirstPress = false;
    public boolean startButtonHeld, backButtonHeld = false;
    public boolean startButtonToggle, backButtonToggle = false;
    public double startButtonTime, backButtonTime = 0;
    public double startButtonCooldown, backButtonCooldown = cooldownTime;

    //main loop, meant to run during every iteration of the teleop.
    //Updates all the variables using one of the teleop's gamepads.
    public void loop(Gamepad gamepad) {
        //Every button has the same code and just uses different base variables.

        //if the leftStick wasn't pressed last iteration but is now,
        //then toggle the left stick variable and set the first press variable to true
        //otherwise, set the firstpress variable to false
        if (!leftStickPressed && gamepad.left_stick_button) {
            leftStickFirstPress = true;
            leftStickToggle = !leftStickToggle;
        }
        else leftStickFirstPress = false;
        //if the leftStick button is pressed and it is the first time being pressed or it has been held longer than the cooldown
        //then set the held variable to true and update the leftStickTime to this iteration's time
        //otherwise set the held variable to false
        if (gamepad.left_stick_button && (!leftStickPressed || time.seconds()- leftStickTime < leftStickCooldown)) {
            leftStickHeld = true;
            leftStickTime = time.seconds();
        } else {
            leftStickHeld = false;
        }
        //set the leftstick pressed variable to match the gamepad
        leftStickPressed = gamepad.left_stick_button;

        if (!rightStickPressed && gamepad.right_stick_button) {
            rightStickFirstPress = true;
            rightStickToggle = !rightStickToggle;
        }
        else rightStickFirstPress = false;
        if (gamepad.right_stick_button && (!rightStickPressed || time.seconds()- rightStickTime < rightStickCooldown)) {
            rightStickHeld = true;
            rightStickTime = time.seconds();
        } else {
            rightStickHeld = false;
        }
        rightStickPressed = gamepad.right_stick_button;

        if (!dpadLeftPressed && gamepad.dpad_left) {
            dpadLeftFirstPress = true;
            dpadLeftToggle = !dpadLeftToggle;
        }
        else dpadLeftFirstPress = false;
        if (gamepad.dpad_left && (!dpadLeftPressed || time.seconds()-dpadLeftTime < dpadLeftCooldown)) {
            dpadLeftHeld = true;
            dpadLeftTime = time.seconds();
        } else {
            dpadLeftHeld = false;
        }
        dpadLeftPressed = gamepad.dpad_left;

        if (!dpadRightPressed && gamepad.dpad_right) {
            dpadRightFirstPress = true;
            dpadRightToggle = !dpadRightToggle;
        }
        else dpadRightFirstPress = false;
        if (gamepad.dpad_right && (!dpadRightPressed ||time.seconds()-dpadRightTime < dpadRightCooldown)) {
            dpadRightHeld = true;
            dpadRightTime = time.seconds();
        } else {
            dpadRightHeld = false;
        }
        dpadRightPressed = gamepad.dpad_right;

        if (!dpadUpFirstPress && gamepad.dpad_up) {
            dpadUpFirstPress = true;
            dpadUpToggle = !dpadUpToggle;
        }
        else dpadUpFirstPress = false;
        if (gamepad.dpad_up && (!dpadUpPressed || time.seconds()-dpadUpTime < dpadUpCooldown)) {
            dpadUpHeld = true;
            dpadUpTime = time.seconds();
        } else {
            dpadUpHeld = false;
        }
        dpadUpPressed = gamepad.dpad_up;

        if (!dpadDownFirstPress && gamepad.dpad_down) {
            dpadDownFirstPress = true;
            dpadDownToggle = !dpadDownToggle;
        }
        else dpadDownFirstPress = false;
        if (gamepad.dpad_down && (!dpadDownPressed || time.seconds()-dpadDownTime < dpadDownCooldown)) {
            dpadDownHeld = true;
            dpadDownTime = time.seconds();
        } else {
            dpadDownHeld = false;
        }
        dpadDownPressed = gamepad.dpad_down;

        if (!aPressed && gamepad.a) {
            aFirstPress = true;
            aToggle = !aToggle;
        }
        else aFirstPress = false;
        if (gamepad.a && (!aPressed || time.seconds()-aTime < aCooldown)) {
            aHeld = true;
            aTime = time.seconds();
        } else {
            aHeld = false;
        }
        aPressed = gamepad.a;

        if (!bPressed && gamepad.b) {
            bFirstPress = true;
            bToggle = !bToggle;
        }
        else bFirstPress = false;
        if (gamepad.b && (!bPressed || time.seconds()-bTime < bCooldown)) {
            bHeld = true;
            bTime = time.seconds();
        } else {
            bHeld = false;
        }
        bPressed = gamepad.b;

        if (!yPressed && gamepad.y) {
            yFirstPress = true;
            yToggle = !yToggle;
        }
        else yFirstPress = false;
        if (gamepad.y && (!yPressed || time.seconds()-yTime < yCooldown)) {
            yHeld = true;
            yTime = time.seconds();
        } else {
            yHeld = false;
        }
        yPressed = gamepad.y;

        if (!xPressed && gamepad.x) {
            xFirstPress = true;
            xToggle = !xToggle;
        }
        else xFirstPress = false;
        if (gamepad.x && (!xPressed || time.seconds()-xTime < xCooldown)) {
            xHeld = true;
            xTime = time.seconds();
        } else {
            xHeld = false;
        }
        xPressed = gamepad.x;

        if (!leftBumperPressed && gamepad.left_bumper) {
            leftBumperFirstPress = true;
            leftBumperToggle = !leftBumperToggle;
        }
        else leftBumperFirstPress = false;
        if (gamepad.left_bumper && (!leftBumperPressed || time.seconds()-leftBumperTime < leftBumperCooldown)) {
            leftBumperHeld = true;
            leftBumperTime = time.seconds();
        } else {
            leftBumperHeld = false;
        }
        leftBumperPressed = gamepad.left_bumper;

        if (!rightBumperPressed && gamepad.right_bumper) {
            rightBumperFirstPress = true;
            rightBumperToggle = !rightBumperToggle;
        }
        else rightBumperFirstPress = false;
        if (gamepad.right_bumper && (!rightBumperPressed || time.seconds()-rightBumperTime < rightBumperCooldown)) {
            rightBumperHeld = true;
            rightBumperTime = time.seconds();
        } else {
            rightBumperHeld = false;
        }
        rightBumperPressed = gamepad.right_bumper;

        if (!backButtonPressed && gamepad.back) {
            backButtonFirstPress = true;
            backButtonToggle = !backButtonToggle;
        }
        else backButtonFirstPress = false;
        if (gamepad.back && (!backButtonPressed || time.seconds()-backButtonTime < backButtonCooldown)) {
            backButtonHeld = true;
            backButtonTime = time.seconds();
        } else {
            backButtonHeld = false;
        }
        backButtonPressed = gamepad.back;

        if (!startButtonPressed && gamepad.start) {
            startButtonFirstPress = true;
            startButtonToggle = !startButtonToggle;
        }
        else startButtonFirstPress = false;
        if (gamepad.start && (!startButtonPressed || time.seconds()-startButtonTime < startButtonCooldown)) {
            startButtonHeld = true;
            startButtonTime = time.seconds();
        } else {
            startButtonHeld = false;
        }
        startButtonPressed = gamepad.start;
    }
}
