// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimSwerve extends SubsystemBase {
  public SimSwerveModule[] Simmodules;
  public SwerveDriveKinematics Simkinematics;
  public SimGyro Simgyro;
  public SwerveDriveOdometry Simodometry;
  public Field2d field;

  public SimSwerve() {
    Simgyro = new SimGyro();
      Simmodules = new SimSwerveModule[]{
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule()
      };
      Simkinematics = Constants.SwerveConstants.swerveKinematics;
      Simodometry = new SwerveDriveOdometry(Simkinematics, Simgyro.getRotation2d(), getSimPositions());
      AutoBuilder.configureHolonomic(
            this::getSimPose, // Robot pose supplier
            this::resetSimOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSimSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveSimRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.01, 0.0, 0.0), // Rotation PID constants
                    3, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );

    field = new Field2d();
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    Simgyro.updateRotation(getSimSpeeds().omegaRadiansPerSecond);
    Simodometry.update(Simgyro.getRotation2d(), getSimPositions());
    field.setRobotPose(getSimPose());
  }

  public void driveSimRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Simkinematics.toSwerveModuleStates(targetSpeeds);
    setSimStates(targetStates);
  }

  @AutoLogOutput(key="robot/SimOdometry")
  public Pose2d getSimPose() {
    return Simodometry.getPoseMeters();
  }

  public void resetSimOdometry(Pose2d pose) {
    Simodometry.resetPosition(Simgyro.getRotation2d(), getSimPositions(), pose);
  }

  public void zeroOdometry(){
    SwerveModulePosition[] zSwerveModulePositions = {new SwerveModulePosition(),
    new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()};
    Simodometry.resetPosition(new Rotation2d(), zSwerveModulePositions, new Pose2d());
  }

  public SwerveModuleState[] getSimModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[Simmodules.length];
    for (int i = 0; i < Simmodules.length; i++) {
      states[i] = Simmodules[i].getState();
    }
    return states;
  }


  public void driveSim(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  {
    SwerveModuleState[] swerveModuleStates =
      Simkinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  -translation.getX(), translation.getY(), rotation, Simgyro.getRotation2d())
              : new ChassisSpeeds(-translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
    setSimStates(swerveModuleStates);
  }

  public SwerveModulePosition[] getSimPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[Simmodules.length];
    for (int i = 0; i < Simmodules.length; i++) {
      positions[i] = Simmodules[i].getPosition();
    }
    return positions;
  }

  public ChassisSpeeds getSimSpeeds() {
    return Simkinematics.toChassisSpeeds(getSimModuleStates());
  }

  public void setSimStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 3);

    for (int i = 0; i < Simmodules.length; i++) {
      Simmodules[i].setTargetState(targetStates[i]);
    }
  }

  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }
}
