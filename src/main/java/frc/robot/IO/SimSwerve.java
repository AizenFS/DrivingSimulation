// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import org.littletonrobotics.junction.AutoLogOutput;


import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SimSwerve {

  private SimSwerveModule[] Simmodules;
  private SwerveDriveKinematics Simkinematics;
  private SimGyro Simgyro;
  private SwerveDriveOdometry Simodometry;
  private Field2d field;

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
    field = new Field2d();
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
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

