package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Assembly;

@TeleOp(name = "Pitch Anlge Get", group = "Test")
public class PitchTest extends LinearOpMode {
    Assembly assembly;
    double pitchAngle;
    double myAngle;

    private double mapPotentiometerToAngle(double potentiometerValue) {
        return ((potentiometerValue - 0.47800000000000004)/ (1.1360000000000001-0.47800000000000004)) * (90 - 0) -0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            assembly = new Assembly(hardwareMap);
        }

        waitForStart();

        while (!isStopRequested()){
            pitchAngle = mapPotentiometerToAngle(assembly.pot.getVoltage());
            myAngle = pitchAngle + 90;

            telemetry.addData("Raw Angle", pitchAngle);
            telemetry.addData("My Angle", myAngle);
        }
    }
}