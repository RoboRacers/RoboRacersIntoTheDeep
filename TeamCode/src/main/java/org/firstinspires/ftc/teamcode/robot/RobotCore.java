package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Subsystem;

import java.util.Arrays;
import java.util.List;

/**
 * This class contains the declaration and setup for all subsystems of the robot.
 */
public class RobotCore implements Subsystem {

    /*
     * Declare the different subsystems of the robot here.
     */

    public List<Subsystem> subsystems;

    /**
     * Constructor for the RobotCore class. Runs the setup for all subsystems.
     * @param hardwareMap
     */
    public RobotCore(HardwareMap hardwareMap){



    }

    /**
     * Updates all subsystems.
     */
    @Override
    public void update() {
        for (Subsystem system: subsystems) system.update();
    }

}
