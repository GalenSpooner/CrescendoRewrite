package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake.IntakeState;
import frc.robot.Subsystems.Shooter.PivotState;

public class Autos {
  
    public static Command fiveNoteAdaptable(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, PathPlannerPath fournote1, PathPlannerPath fournote2, PathPlannerPath fournote3, PathPlannerPath fournote4, PathPlannerPath fournote5, PathPlannerPath fivenote1, PathPlannerPath fivenote2, PathPlannerPath fivenotealt, PathPlannerPath fivenotealt2){
        return new SequentialCommandGroup(

          shooter.setState(PivotState.SPEAKER),
          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fournote1),
          drivetrain.runPathplannerPathFile(fournote2),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),
          
          drivetrain.runPathplannerPathFile(fournote3),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fournote4),
          drivetrain.runPathplannerPathFile(fournote5),

          shooter.ShootSpeaker(),
          intake.setState(IntakeState.INTAKING),

          drivetrain.runPathplannerPathFile(fivenote1),

          //logic for if robot does not pick up a note (it will go to the next note)

          ((intake.getState() == IntakeState.HOLDING) ? new InstantCommand(() ->{
            new SequentialCommandGroup(
            drivetrain.runPathplannerPathFile(fivenote2),
            shooter.ShootSpeaker()
            );

          }) : new InstantCommand(() -> {
            new SequentialCommandGroup(
            drivetrain.runPathplannerPathFile(fivenotealt),
            drivetrain.runPathplannerPathFile(fivenotealt2),
            shooter.ShootSpeaker()
            );
          }))
        );
    }
    
    public static Command sixNote(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, PathPlannerPath fournote1, PathPlannerPath fournote2, PathPlannerPath fournote3, PathPlannerPath fournote4, PathPlannerPath fournote5, PathPlannerPath fivenote1, PathPlannerPath fivenote2, PathPlannerPath fivenotealt, PathPlannerPath fivenotealt2, PathPlannerPath sixnote1, PathPlannerPath sixnote2){
        return new SequentialCommandGroup(
            fiveNoteAdaptable(drivetrain, shooter, intake, fournote1, fournote2, fournote3, fournote4, fournote5, fivenote1, fivenote2, fivenotealt, fivenotealt2),
            intake.setState(IntakeState.INTAKING),
          drivetrain.runPathplannerPathFile(sixnote1),

          //dont bother to do anything if 6th note wasn't picked up

          ((intake.getState() == IntakeState.HOLDING)? new InstantCommand(() -> {
            drivetrain.runPathplannerPathFile(sixnote2);
            shooter.ShootSpeaker();
          }): new InstantCommand())
            );
    }
}

