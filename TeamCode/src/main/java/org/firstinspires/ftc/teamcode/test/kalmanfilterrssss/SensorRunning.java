package org.firstinspires.ftc.teamcode.test.kalmanfilterrssss;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.test.kalmanfilterrssss.kalmanfilter;

@TeleOp(name="IMU Test", group="Linear Opmode")
public class SensorRunning extends LinearOpMode {
    kalmanfilter KalmanFilter;
    private ElapsedTime timer = new ElapsedTime();
    private IMU imuSensor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void runOpMode(){
    
    }
}
