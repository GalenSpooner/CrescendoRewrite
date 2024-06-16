// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  SendableChooser chooser = new SendableChooser<>();
  CommandSwerveDrivetrain drivetrain;
  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    
    SmartDashboard.putData(chooser);
    chooser.addOption("OnePiece", 0);
    chooser.addOption("TwoPiece", 1);
    
    configureBindings();
  }

  private void configureBindings() {}
  private Command chooseAuto(){
    switch ((int)chooser.getSelected()) {
      case 0:
        return null;
    
      default:
        return null;
    }
  }
  public Command getAutonomousCommand() {
    return chooseAuto();
  }
}
