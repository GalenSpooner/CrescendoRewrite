package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import RockinLib.MotorControllers.RockinSparkMax;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    DoubleSolenoid climber1;
    DoubleSolenoid climber2;
    public Climber(){
        climber1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
        climber2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
        
    }

    public Command climbersUp(){
        return run(() -> {
            climber1.set(Value.kForward);
            climber2.set(Value.kForward);
            
        }
        );
    }
    public Command climbersDown(){
        return run(() -> {
            climber1.set(Value.kReverse);
            climber2.set(Value.kReverse);
        }
        );
    }
    
}
