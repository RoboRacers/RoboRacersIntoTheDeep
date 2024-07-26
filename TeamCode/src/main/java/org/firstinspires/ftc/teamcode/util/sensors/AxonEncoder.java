package org.firstinspires.ftc.teamcode.util.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AxonEncoder {

    private AnalogInput analogInput;

    public AxonEncoder(AnalogInput input) {
        analogInput = input;
    }

    public double getReadingDegrees() {
        return analogInput.getVoltage() / 3.3 * 360;
    }

}
