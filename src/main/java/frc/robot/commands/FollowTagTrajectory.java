package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTagTrajectory extends Command{
  /** Creates a new FollowTagTrajectory. */
  LimeLight limelight  = new LimeLight(); 
  TrajectoryConfig config;
  private SwerveSubsystem swerve = new SwerveSubsystem();
  public FollowTagTrajectory() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond,
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.SwerveConstants.swerveKinematics);
    
    
      //loadCommand(limelight.generateTargetTrajectory(config));
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Command loadCommand(Trajectory trajectory) {
    var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
      trajectory, // PUT TRAJECTORY HERE
      swerve::getPose,
      Constants.SwerveConstants.swerveKinematics,
      new PIDController(Constants.AutoConstants.kPXController, 0, 0),
      new PIDController(Constants.AutoConstants.kPYController, 0, 0),
      thetaController,
      swerve::setModuleStates,
      swerve);
        
      return new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())).andThen(swerveControllerCommand);
  }
  
}