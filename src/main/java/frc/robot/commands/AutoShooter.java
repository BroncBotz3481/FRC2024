// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder.FeederSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;


public class AutoShooter extends Command {

  private ShooterSubsystem m_shooter;
  private FeederSubsystem m_feeder;
  private Timer timer;
  /** Creates a new AutoShooter. */
  public AutoShooter(ShooterSubsystem shooter, FeederSubsystem feeder) {
    m_shooter = shooter;
    m_feeder = feeder;
    timer = new Timer();
    addRequirements(m_shooter);
    addRequirements(m_feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shoot(0);
    m_feeder.setSpeed(0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(timer.get());
    m_shooter.runPID(-5500);
    if (timer.get()>=2)
      m_feeder.setSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shoot(0);
    m_feeder.setSpeed(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get()>=4.5;
  }
}
