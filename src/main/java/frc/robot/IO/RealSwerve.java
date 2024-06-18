// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IO;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveModule;

public class RealSwerve {

  public final AHRS gyro;
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;

  public Field2d field;

  public RealSwerve() {
    gyro = new AHRS(SPI.Port.kMXP);
    zeroGyro();
    mSwerveMods =
    new SwerveModule[] {
      new SwerveModule(0, Constants.SwerveConstants.Mod1.constants),
      new SwerveModule(1, Constants.SwerveConstants.Mod0.constants),
      new SwerveModule(2, Constants.SwerveConstants.Mod3.constants),
      new SwerveModule(3, Constants.SwerveConstants.Mod2.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  //takes the coordinate on field wants to go to, the rotation of it, whether or not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates =
      Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  -translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(-translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  @AutoLogOutput(key="odometry/Robot")
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }
 
  public ChassisSpeeds getSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  public void driveRobotRelitive(ChassisSpeeds targetSpeed)
  {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeed,
        new Translation2d());
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);// MIGHT NEED TO CHANGE
    }
  }
  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
      // front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      // front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      // back left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
      // back right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }

  @AutoLogOutput
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  @AutoLogOutput
  public Rotation2d getYaw() {
    //fancy if else loop again
    return (Constants.SwerveConstants.invertNavx)
        ? Rotation2d.fromDegrees(-(360 - gyro.getYaw()))
        : Rotation2d.fromDegrees(-(gyro.getYaw() + Constants.SwerveConstants.degreesOffSet));
  }

  public boolean AutoBalance(){
    double roll_error = gyro.getPitch();//the angle of the robot
    double balance_kp = -.005;//Variable muliplied by roll_error
    double position_adjust = 0.0;
    double min_command = 0.0;//adds a minimum input to the motors to overcome friction if the position adjust isn't enough
    if (roll_error > 6.0)
    {
      position_adjust = balance_kp * roll_error + min_command;//equation that figures out how fast it should go to adjust
      //position_adjust = Math.max(Math.min(position_adjust,.15), -.15);  this gets the same thing done in one line
      if (position_adjust > .1){position_adjust = .1;}
      if (position_adjust < -.1){position_adjust = -.1;}
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      
      return false;
    }
    else if (roll_error < -6.0)
    {
      position_adjust = balance_kp * roll_error - min_command;
      drive(new Translation2d(position_adjust, 0), 0.0, true, false);
      if (position_adjust > .3){position_adjust = .3;}
      if (position_adjust < -.3){position_adjust = -.3;}
      return false;
    }
    else{
      drive(new Translation2d(0, 0), 0.0, true, false);
      return true;}
  }
}
