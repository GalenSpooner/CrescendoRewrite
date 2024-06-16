package frc.robot;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
public class ShooterTarget {
    InterpolatingDoubleTreeMap angleMap;
    InterpolatingDoubleTreeMap velocityMap;
    public class targetValues{
        public double velocity;
        public double angle;
        public targetValues(double velocity, double angle){
            this.velocity = velocity;
            this.angle = angle;
        }
    }
    public ShooterTarget(double[] distances, double[] angles, double[] velocities){
        angleMap = new InterpolatingDoubleTreeMap();
        velocityMap = new InterpolatingDoubleTreeMap();
        if(distances.length == angles.length && distances.length == angles.length){
            for(int x = 0; x < distances.length; x++){
                angleMap.put(distances[x], angles[x]);
            }
            for(int x = 0; x < distances.length; x++){
                velocityMap.put(distances[x], velocities[x]);
            }
        }
    }
    public targetValues calculateFromDistance(double distance){
        return new targetValues(velocityMap.get(distance),angleMap.get(distance));
        
    }
}
