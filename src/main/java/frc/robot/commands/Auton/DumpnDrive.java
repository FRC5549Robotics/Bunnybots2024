// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shintake;
import frc.robot.RobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

public class DumpnDrive extends SequentialCommandGroup {
  /** Creates a new DumpnDrive. */
  public DumpnDrive(Shintake shintake, PathPlannerPath traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new InstantCommand(shintake::shintake_back),
      AutoBuilder.followPath(traj)
    );
  }

}
