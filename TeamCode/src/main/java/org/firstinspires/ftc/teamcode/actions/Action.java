package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;


@FunctionalInterface
public interface Action {
    /**
     * Runs a single uninterruptible block. Returns true if the action should run again and false if it has completed.
     * A telemetry packet [p] is provided to record any information on the action's progress.
     */
    boolean run(TelemetryPacket p);

    /**
     * Draws a preview of the action on canvas [fieldOverlay].
     */
    default void preview(Canvas fieldOverlay) {}
}

