package pathfollower.follower;

import com.roboracers.topgear.geometry.Pose2d;
import com.roboracers.topgear.planner.ParametricPath;

public interface Follower {

    void setPath(ParametricPath parametricPath);

    Pose2d getDriveVelocity(Pose2d currentPosition);

    Boolean isComplete(Pose2d currentPosition);
}
