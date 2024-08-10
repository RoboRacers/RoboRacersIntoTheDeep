package org.firstinspires.ftc.teamcode.actions;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class SleepAction implements Action {
    private double dt;
    private double beginTs = -1.0;

    public SleepAction(double dt) {
        this.dt = dt;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        double t;
        if (beginTs < 0) {
            beginTs = now();
            t = 0.0;
        } else {
            t = now() - beginTs;
        }

        return t < dt;
    }

    @Override
    public void preview(Canvas fieldOverlay) {}

    private double now() {
        // Implement the method to return the current time in seconds
        return System.nanoTime() * 1e-9;
    }

}