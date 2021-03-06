package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Autonomous
public class BatteryTester extends OpMode {
    DcMotorEx[] motors = new DcMotorEx[0];
    DataLogger Dl;
    List<LynxModule> allHubs;
    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        ArrayList<String> headers = new ArrayList<>();
        headers.add("System Time");
        for (LynxModule hub : allHubs) {
            headers.add(hub.getSerialNumber() + " Input voltage");
            headers.add(hub.getSerialNumber() + " Auxilary voltage");
            headers.add(hub.getSerialNumber() + " Current");
        }
        motors = new DcMotorEx[hardwareMap.dcMotor.size()];
        int curIter = 0;
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            motors[curIter] = (DcMotorEx)entry.getValue();
            headers.add(entry.getKey() + " Amperage");
            curIter++;
        }
        try {
            // Create Datalogger
            Dl = new DataLogger( "battery tester" + System.currentTimeMillis() + ".csv");

            Dl.addHeaderLine(headers.toArray(new String[headers.size()]));

            // Update the log file
        } catch (IOException e){

        }
    }

    @Override
    public void loop() {
        ArrayList<Object> dataLine = new ArrayList<>();
        dataLine.add(System.currentTimeMillis());
        telemetry.addData("Runtime", getRuntime());
        for (LynxModule hub : allHubs) {
            dataLine.add(hub.getInputVoltage(VoltageUnit.VOLTS));
            dataLine.add(hub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            dataLine.add(hub.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(hub.getSerialNumber() + " inputVoltage", hub.getInputVoltage(VoltageUnit.VOLTS));
            telemetry.addData(hub.getSerialNumber() + "auxilaryVoltage", hub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
            telemetry.addData(hub.getSerialNumber() + " current", hub.getCurrent(CurrentUnit.AMPS));
        }
        for (int i = 0; i < motors.length; i++) {
            dataLine.add(motors[i].getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor " + i + " current", motors[i].getCurrent(CurrentUnit.AMPS));
            motors[i].setPower(1);
        }
        telemetry.update();
        try {
            Dl.addDataLine(dataLine.toArray());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void stop() {
        Dl.close();
    }
}
