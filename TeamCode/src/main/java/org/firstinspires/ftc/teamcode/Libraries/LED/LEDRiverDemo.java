package org.firstinspires.ftc.teamcode.Libraries.LED;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LEDRiver Demo")
public class LEDRiverDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxModule revHub = hardwareMap.get(LynxModule.class, "Control Hub");
        try {
            new LynxI2cConfigureChannelCommand(revHub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K).send();
        } catch (LynxNackException | InterruptedException ex) {
            ex.printStackTrace();
        }

        LEDRiver ledRiver = hardwareMap.get(LEDRiver.IMPL, "ledriver");
        ledRiver.setLEDCount(400);
        ledRiver.setMode(LEDRiver.Mode.SOLID);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGBW);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_8);
        ledRiver.setBrightness(1);
        ledRiver.setColor(0, new LEDRiver.Color(255, 0, 0, 0));
        ledRiver.setColor(1, new LEDRiver.Color(0,255,0,0));
        ledRiver.setColor(2, new LEDRiver.Color(0,0,0,0));

        telemetry.addData("ready: ", "true");
        telemetry.update();
        waitForStart();

        ledRiver.apply();

        Thread.sleep(1000);
        ledRiver.setColor(new LEDRiver.Color(0,0,255,0)).apply();
        Thread.sleep(1000);

        ledRiver.setColor(new LEDRiver.Color(0,255,0,0)).apply();
        Thread.sleep(1000);

        ledRiver.setColor(new LEDRiver.Color(255,0,0,0)).apply();
        Thread.sleep(1000);

        ledRiver.setMode(LEDRiver.Mode.PATTERN).setColor(new LEDRiver.Color(255,0,0,0));
        ledRiver.setPattern(LEDRiver.Pattern.STROBE.builder());
        ledRiver.apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.HEARTBEAT.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.BREATHING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.RUNNING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.BOUNCING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_RUNNING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.THEATRE_BOUNCING.builder()).apply();
        Thread.sleep(5000);

        ledRiver.setPattern(LEDRiver.Pattern.COLOR_WHEEL.builder()).apply();
        ledRiver.save();
        Thread.sleep(5000);

        ledRiver.setHide(true).apply();
        Thread.sleep(2000);

        ledRiver.setHide(false);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_8);
        ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        long end_time = System.currentTimeMillis() + 5000;
        int shift = 0;
        while(System.currentTimeMillis() < end_time) {
            shift = (shift + 5) % 360;
            for(int i = 0; i < 60; i++) {
                ledRiver.setColor(i, Color.HSVToColor(new float[] {(i*5+shift)%360, 1, 1}));
            }
            ledRiver.apply();
        }
        ledRiver.setMode(LEDRiver.Mode.PATTERN);
        ledRiver.setColor(0, new LEDRiver.Color(0,0,0,255));
        ledRiver.setColor(1, new LEDRiver.Color(0,0,255,0));
        ledRiver.setColor(2, new LEDRiver.Color(0,0,0,0));
        ledRiver.setPattern(LEDRiver.Pattern.BOUNCING.builder().setLength(5).setSpeed(20)).apply();
        Thread.sleep(5000);
        ledRiver.setLEDMode(LEDRiver.LEDMode.RGBW);
        ledRiver.setMode(LEDRiver.Mode.SOLID).setColor(new LEDRiver.Color(0,0,0,0)).apply();
        Thread.sleep(1000);
        ledRiver.setHide(false);
        ledRiver.setColorDepth(LEDRiver.ColorDepth.BIT_8);
        ledRiver.setMode(LEDRiver.Mode.INDIVIDUAL);
        for (int i = 0; i < 104; i++) {
            ledRiver.setColor(i, i < 104/3 ? new LEDRiver.Color(0,0,0,255) : new LEDRiver.Color(0,0,0,0));
        }
        ledRiver.apply();
        Thread.sleep(1000);
        for (int i = 0; i < 199; i++) {
            ledRiver.setColor(i, i < 104/3*2 ? new LEDRiver.Color(0,0,0,255) : new LEDRiver.Color(0,0,0,0));
        }
        ledRiver.apply();
        Thread.sleep(1000);
        for (int i = 0; i < 104; i++) {
            ledRiver.setColor(i, new LEDRiver.Color(0,0,0,255));
        }
        ledRiver.apply();
        Thread.sleep(2000);
        ledRiver.reset();
        while (opModeIsActive());
    }
}
