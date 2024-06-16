package frc.robot.Subsystems;

import RockinLib.MotorControllers.RockinTalon;
import RockinLib.Sensors.RockinCancoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterTarget;

public class Shooter extends SubsystemBase  {
    RockinTalon shooterOne;
    RockinTalon shooterTwo;
    RockinTalon feederOne;
    RockinTalon feederTwo;
    RockinTalon pivot;
    RockinCancoder encoder;
    ShooterTarget target;
    public Shooter(){
        
    }

}
