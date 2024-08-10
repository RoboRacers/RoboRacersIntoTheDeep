package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction implements Action {
    private List<Action> initialActions;
    private List<Action> actions;

    public ParallelAction(List<Action> initialActions) {
        this.initialActions = initialActions;
        this.actions = new ArrayList<>(initialActions);
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        actions.removeIf(action -> !action.run(p));
        return !actions.isEmpty();
    }

    @Override
    public void preview(Canvas fieldOverlay) {
        for (Action a : initialActions) {
            a.preview(fieldOverlay);
        }
    }
}

