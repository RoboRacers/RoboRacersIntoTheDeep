package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="MaxBotix Ultrasonic Sensor", group="LinearOpMode")
public class sensorfusiontest extends LinearOpMode {

    private AnalogInput ultrasonicSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public MovingAverageUltraSensor movingUltra;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "sensor_range");
        movingUltra = new MovingAverageUltraSensor(ultrasonicSensor);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double distance = movingUltra.readDistance();
            double averageDistance = movingUltra.getMovingAverage();

            // Update telemetry
            telemetry.addData("Distance", "%.2f cm", distance);
            telemetry.addData("Average Distance", "%.2f cm", averageDistance);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Update dashboard telemetry
            packet.put("Distance", distance);
            packet.put("Average Distance", averageDistance);
            packet.put("Runtime", runtime.toString());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();

            // Always call idle() at the end of your loop to give the system a chance to breathe
            idle();
        }
    }
}