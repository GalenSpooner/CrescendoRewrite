package RockinLib.Sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RockinCancoder extends CANcoder implements Subsystem{
    String name;
    public RockinCancoder(int canId, String canBus, String name){
        super(canId,canBus);
        this.name = name;
        
    }
    public double getDegrees(){
        return this.getAbsolutePosition().getValueAsDouble();
    }
    public void attachToTalonFX(TalonFX talonFX){
        talonFX.getConfigurator().apply(new TalonFXConfiguration().Feedback.withRemoteCANcoder(this));
    }
    public void getVelocityInRPS(){
        this.getVelocity().getValueAsDouble();
    }
    @Override
    public String getName() {
        return name;
    }
    @Override
    public void periodic() {
       SmartDashboard.putNumber(this.getName() + " position (Degrees)", getDegrees());
    }

} 