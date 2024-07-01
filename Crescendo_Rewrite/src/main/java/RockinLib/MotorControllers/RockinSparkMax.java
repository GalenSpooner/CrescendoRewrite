package RockinLib.MotorControllers;

import com.revrobotics.CANSparkMax;

//i hate rev
public class RockinSparkMax extends CANSparkMax {
    public RockinSparkMax(int CanID, MotorType type ){
        super(CanID, type);
    
    }
    public void setCurrentLimit(int CurrentLimit){
        this.setSmartCurrentLimit(CurrentLimit);
    }
    
}
