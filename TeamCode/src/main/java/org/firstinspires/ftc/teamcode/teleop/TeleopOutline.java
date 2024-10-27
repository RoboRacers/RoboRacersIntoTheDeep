package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.useless.TankDrive;

@TeleOp(name ="TeleopOutline", group = "16481-IntoTheDeep")
public class TeleopOutline extends LinearOpMode {
    public DcMotorImplEx intakeMotor;
    public ServoImplEx flipLeft;
    public CRServoImplEx rolling;
    public ServoImplEx flipRight;
    public DcMotorImplEx slideMotor;  // New slide motor for vertical movement

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
            flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
            intakeMotor = hardwareMap.get(DcMotorImplEx.class, "intakeRoll");
            rolling = hardwareMap.get(CRServoImplEx.class, "rolling");
            slideMotor = hardwareMap.get(DcMotorImplEx.class, "slideMotor");  // Initialize slide motor

            waitForStart();

            while (opModeIsActive()) {
                // Flip Left and Right Controls
                if (gamepad1.a) {
                    flipLeft.setPosition(0.7);
                } else if (gamepad1.b) {
                    flipRight.setPosition(0.3);
                } else if (gamepad1.right_trigger > 0.1) {
                    rolling.setPower(0.5);
                }

                // Intake Motor Control
                intakeMotor.setPower(gamepad1.right_stick_y);

                // Slide Vertical Control
                if (gamepad1.x) {
                    slideMotor.setPower(1.0);  // Slide up
                } else if (gamepad1.y) {
                    slideMotor.setPower(-1.0);  // Slide down
                } else {
                    slideMotor.setPower(0);  // Stop slide motor when no button is pressed
                }

                // Additional Flip Sequence for Deposit
                if (gamepad1.left_bumper) {
                    flipLeft.setPosition(0.5);  // Start position for deposit flip
                    flipRight.setPosition(0.5);  // Mid position
                    flipLeft.setPosition(1.0);  // Final deposit position
                }

                // Update Pose Estimate
                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}