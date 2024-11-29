package org.firstinspires.ftc.teamcode.teleop.test;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.PIDController;

@Config
@TeleOp(name = "Slides PID Test", group = "Test")
public class SlidesPIDTest extends LinearOpMode {
    public DcMotorImplEx slidesMotor;

//    public static double kG = 0.27;
    public static double kP = 0.001;
    public static  double kI = 0;
    public static  double kD = 0.0000;
    public static double target = 150;  //min: 0 max: 1930

//    public static double offset = 40;
    public static double error = 0;
    public static double offset = 40;


//    public static double ticksPerRightAngle = 930;

    PIDController slidesControl;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");

        slidesMotor.setDirection(DcMotorImplEx.Direction.REVERSE);



        slidesControl = new PIDController(kP, kI, kD);

//        final double ticksToDegrees = (double) 90 /ticksPerRightAngle;

        while (opModeInInit()) {

        }

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

            slidesControl.setCoefficients(kP, kI, kD);

            if (gamepad1.triangle) {
                target = 300;
            } else if (gamepad1.cross) {
                target = 200;
            } else if (gamepad1.circle) {
                target = 150;
            }

            slidesControl.setSetpoint(target);

//            double feedforward = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;

            double pid = slidesControl.calculate(slidesMotor.getCurrentPosition());

            slidesMotor.setPower(pid);

            // Compute error as the difference between current position and target
            double currentPosition = slidesMotor.getCurrentPosition();
            error = target - currentPosition;
            
//            telemetry.addData("Feedforward", feedforward);
            telemetry.addData("Target", target);
            telemetry.addData("Error", error);
            telemetry.addData("Pitch Motor Position", slidesMotor.getCurrentPosition());
            telemetry.addData("Pitch Motor Power", slidesMotor.getPower());
            telemetry.addData("Pitch Current", slidesMotor.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetry.addData("Pitch Motor Angle", (pitchMotor.getCurrentPosition() - 45 ) * ticksToDegrees);
            telemetry.update();

        }
    }
}
