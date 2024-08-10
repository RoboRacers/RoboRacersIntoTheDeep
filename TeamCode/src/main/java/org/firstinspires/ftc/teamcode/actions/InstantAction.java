package org.firstinspires.ftc.teamcode.actions;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Instant action that executes the given InstantFunction immediately.
 */
public class InstantAction implements Action {
    private final InstantFunction f;

    public InstantAction(InstantFunction f) {
        this.f = f;
    }

    @Override
    public boolean run(TelemetryPacket p) {
        f.run();
        return false;
    }

    @FunctionalInterface
    public interface InstantFunction {
        void run();
    }
}