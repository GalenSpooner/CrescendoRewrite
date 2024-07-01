package RockinLib.Control;

import edu.wpi.first.math.geometry.Pose2d;

public class RockinField {
    public Pose2d swapToRed(Pose2d bluepose){
        
        return new Pose2d(54 - bluepose.getX(), bluepose.getY(), bluepose.getRotation());
    }
}
