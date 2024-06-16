package RockinLib.Control;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class RockinRio extends SubsystemBase{
    public RockinRio(double brownOutVoltage){
        RobotController.setBrownoutVoltage(brownOutVoltage);
    }
    public double getCanivoreUtilization(String canivoreName){
        return CANBus.getStatus(canivoreName).BusUtilization * 100;
    }
    public double getRioCanUtilization(){
        return RobotController.getCANStatus().percentBusUtilization;
    }
    public boolean isBrowningOut(){
        return RobotController.isBrownedOut();
    }
    public void setBrownout(double brownoutVoltage){
        RobotController.setBrownoutVoltage(brownoutVoltage);
    }
    public void periodic(){
        SmartDashboard.putNumber("Canivore utilization", getCanivoreUtilization("canivore"));
         SmartDashboard.putNumber("Rio Can utilization", getRioCanUtilization());
        SmartDashboard.putBoolean("Is Browning Out",isBrowningOut());
    }
}
