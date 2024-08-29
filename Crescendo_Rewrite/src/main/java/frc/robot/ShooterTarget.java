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
    // given a list of distances shot from and angles and velocities that work for that shot, interpolate into a function so that you can plug in your distance and get th velocity and angle for that shot
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
