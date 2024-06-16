package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import RockinLib.MotorControllers.RockinSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    RockinSparkMax leftClimber;
    RockinSparkMax rightClimber;
    public Climber(){
        leftClimber = new RockinSparkMax(Constants.ClimberConstants.CLIMBER_LEFT_ID,MotorType.kBrushless);
        rightClimber = new RockinSparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);
        
    }
    private void setLeftClimber(double DutyCycleOut){
        leftClimber.set(DutyCycleOut);
    }
    private void setRightClimber(double DutyCycleOut){
        rightClimber.set(DutyCycleOut);
    }

    public Command climbersUp(){
        return run(() -> {
            setLeftClimber(Constants.ClimberConstants.CLIMBER_LEFT_DUTYCYCLE);
            setRightClimber(Constants.ClimberConstants.CLIMBER_RIGHT_DUTYCYCLE);
        }
        );
    }
    public Command climbersDown(){
        return run(() -> {
            setLeftClimber(-Constants.ClimberConstants.CLIMBER_LEFT_DUTYCYCLE);
            setRightClimber(-Constants.ClimberConstants.CLIMBER_RIGHT_DUTYCYCLE);
        }
        );
    }
    
}
