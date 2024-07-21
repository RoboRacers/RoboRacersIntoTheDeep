package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

@Config
@Disabled
@TeleOp(name = "Teleop for Demo", group = "16481-Centerstage")
public class TeleopPIDToPoint extends LinearOpMode {

    RobotCore robot;

    PIDFController xPID =  new PIDFController(new PIDCoefficients(0,0,0));
    PIDFController yPID =  new PIDFController(new PIDCoefficients(0,0,0));
    PIDFController headingPID =  new PIDFController(new PIDCoefficients(0,0,0));

    public static double xTarget = 0;
    public static double yTarget = 0;
    public static double headingTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        Gamepad previousGamepad1 = gamepad1;
        Gamepad previousGamepad2 = gamepad2;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();


        boolean test = false;

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            previousGamepad1 = currentGamepad1;
            previousGamepad2 = currentGamepad2;

            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

            xPID.setTargetPosition(xTarget);
            yPID.setTargetPosition(yTarget);
            headingPID.setTargetPosition(Math.toRadians(headingTarget));

            Pose2d currentPose = robot.drive.getPoseEstimate();

            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            xPID.update(currentPose.getX()),
                            yPID.update(currentPose.getY()),
                            headingPID.update(currentPose.getHeading())
                    )
            );

            // Update all state machines
            robot.update();

            // Telemetry
            telemetry.addLine("\uD83C\uDFCE PID to Point Implementation");
            telemetry.update();
        }
    }
}
