package pathfollower.planner;

import com.roboracers.topgear.geometry.Vector2d;

public interface ParametricPath {

    Vector2d getPoint(double t);
    Vector2d getDerivative(double t);

    double getArcLength();

}
