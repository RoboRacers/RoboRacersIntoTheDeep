//package org.firstinspires.ftc.teamcode.teleop.test;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.modules.PIDController;
//import org.firstinspires.ftc.teamcode.robot.Subsystem;
//
////@TeleOp(name = "LM2 One Driver", group = "Test")
//public class OneDriverActions2 implements Subsystem {
//    //Pitch Stuff
//
//    Servo uno;
//    Servo dos;
//
//    public static double flipPos = 0;
//
//        public  double kG2 = 0.027;
//    public  double kG = 0.001;
//        public DcMotorImplEx slidesMotor;
//        public DcMotorImplEx pitchMotor;
//    public static double kP = 0.005;
//    public static  double kI = 0;
//    public static  double kD = 0.00045;
//        public  double offset = 40;
//        PIDController pitchControl;
//
//        public  double target = 0; //slides
//        public  double ticksPerMaxExtend = 1936;
//        public  double ticksPerRightAngle = 930;
//    .
//        public static double target2 = 100; // flip
//
//
//    public void Slides(HardwareMap hardwareMap) {
//            pitchMotor = hardwareMap.get(DcMotorImplEx.class, "pitchMotor");
//            slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");
//            uno = hardwareMap.get(Servo.class, "flipLeft");
//            dos = hardwareMap.get(Servo.class, "flipRight");
//            pitchMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//            pitchMotor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);;
//            pitchControl = new PIDController(kP, kI, kD);
//        }
//
//
//    public class slidesUp implements Action{
//        private boolean initialized = false;
//
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if (!initialized) {
//                target2 = 1650;
//                pitchControl.setCoefficients(kP, kI, kD);
//                pitchControl = new PIDController(kP, kI, kD);
//                final double ticksToInches = (double) 26 /ticksPerMaxExtend;
//                final double ticksToDegrees = (double) 90 /ticksPerRightAngle;
//                pitchControl.setSetpoint(target);
//                double feedforward2 = kG2 * Math.sin(Math.toRadians((slidesMotor.getCurrentPosition()) * ticksToInches)) + 0;
//                double feedforward3 = kG * Math.cos(Math.toRadians((pitchMotor.getCurrentPosition() - offset) * ticksToDegrees)) + 0;
//                double pid = pitchControl.calculate(pitchMotor.getCurrentPosition());
//                pitchMotor.setPower(feedforward3 + pid + feedforward2);
//                initialized = true;
//            }
//
//            double pos = pitchMotor.getCurrentPosition();
//            return pos>1620 && pos<1680;
//        }
//
//    }
//    public Action extendSlide() {
//        return new OneDriverActions.slidesUp();
//    }
//    @Override
//    public void update() {
//
//    }
//
//
//
//}
