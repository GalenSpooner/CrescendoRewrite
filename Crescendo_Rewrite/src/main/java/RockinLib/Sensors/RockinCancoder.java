package RockinLib.Sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RockinCancoder extends CANcoder implements Subsystem{
    String name;
    public RockinCancoder(int canId, String canBus, String name, double offset, boolean clockwisePositive){
        super(canId,canBus);
        this.name = name;
        this.getConfigurator().refresh(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(offset).withSensorDirection((clockwisePositive)? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)));
    }
    public double getDegrees(){
        return Units.rotationsToDegrees(this.getAbsolutePosition().getValueAsDouble());
    }
    /**
     *  Method to have a TalonFX use this cancoder instead of it's internal encoder
     * @param talonFX that you want to bind this cancoder to
     */
    public void attachToTalonFX(TalonFX talonFX,double encoderToMechRatio, double gearRatio){
        talonFX.getConfigurator().refresh(new TalonFXConfiguration().Feedback.withRemoteCANcoder(this).withSensorToMechanismRatio(encoderToMechRatio).withRotorToSensorRatio(gearRatio));
        
    }
    public double getVelocityInRPS(){
        return this.getVelocity().getValueAsDouble();
    }
    /**
 * Gives name of Cancoder
 * @return Cancoder name
  */
    @Override
    public String getName() {
        return name;
    }
    @Override
    public void periodic() {
       SmartDashboard.putNumber(this.getName() + " position (Degrees)", getDegrees());
    }

} 