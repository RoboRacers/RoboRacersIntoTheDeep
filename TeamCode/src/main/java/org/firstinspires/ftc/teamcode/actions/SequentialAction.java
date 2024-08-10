package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;

public class SequentialAction implements Action {
    private List<Action> initialActions;
    private List<Action> actions;

    public SequentialAction(List<Action> initialActions) {
        this.initialActions = initialActions;
        this.actions = new ArrayList<>(initialActions);
    }

    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public boolean run(TelemetryPacket p) {
        if (actions.isEmpty()) {
            return false;
        }

        if (actions.get(0).run(p)) {
            return true;
        } else {
            actions.remove(0);
            return run(p);
        }
    }

    @Override
    public void preview(Canvas fieldOverlay) {
        for (Action a : initialActions) {
            a.preview(fieldOverlay);
        }
    }
}

