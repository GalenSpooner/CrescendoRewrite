package RockinLib.MotorControllers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class RockinTalon extends TalonFX {
    TalonFXConfigurator configurator;
    public RockinTalon(int deviceId, double supplyCurrentLimit) {
        super(deviceId);
        this.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(supplyCurrentLimit).withSupplyCurrentLimitEnable(true)));
        
    }
    public RockinTalon(int deviceId){
        super(deviceId);
    }

    @Override
    public void set(double speed){
        setControl(new DutyCycleOut(speed,false,false,false,false));
    }
    
}
