package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.drive.ThreeTrackingWheelLocalizer;
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
    public MecanumDrive drive;

    public List<Subsystem> subsystems;

    /**
     * Constructor for the RobotCore class. Runs the setup for all subsystems.
     * @param hardwareMap
     */
    public RobotCore(HardwareMap hardwareMap){

        /* Initialize Subsystems */
        drive = new MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //vision = new Vision(hardwareMap);

        subsystems = Arrays.asList(
                drive
        );

    }

    /**
     * Updates all subsystems.
     */
    @Override
    public void update() {
        for (Subsystem system: subsystems) system.update();
    }

    public void telemetrySelfCheck(Telemetry telemetry) {
        telemetry.addLine("SELF CHECK -----");

        // Checks if the positions of the encoders to make sure they are not unplugged
        drive.updatePoseEstimate();
        ThreeTrackingWheelLocalizer localizer = (ThreeTrackingWheelLocalizer) drive.getLocalizer();
        List<Double> deadwheelPositions = localizer.getWheelPositions();

        telemetry.addData("Left Encoder Pos", deadwheelPositions.get(0));
        telemetry.addData("Right Encoder Pos", deadwheelPositions.get(1));
        telemetry.addData("Perpendicular Encoder Pos", deadwheelPositions.get(2));

        if (deadwheelPositions.get(0) == 0) {
            telemetry.addLine("LEFT ENCODER UNPLUGGED, Check wiring of Port x");
        }
        if (deadwheelPositions.get(1) == 0) {
            telemetry.addLine("RIGHT ENCODER UNPLUGGED, Check wiring of Port x");
        }
        if (deadwheelPositions.get(2) == 0) {
            telemetry.addLine("PERPENDICULAR ENCODER UNPLUGGED, Check wiring of Port x");
        }
    }
}
