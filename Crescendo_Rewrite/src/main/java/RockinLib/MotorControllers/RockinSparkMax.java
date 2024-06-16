package RockinLib.MotorControllers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Current;

//i hate rev
public class RockinSparkMax extends CANSparkMax {
    public RockinSparkMax(int CanID, MotorType type ){
        super(CanID, type);
    
    }
    public void setCurrentLimit(int CurrentLimit){
        this.setSmartCurrentLimit(CurrentLimit);
    }
    
}
