// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO.RealSwerve;

public class SwerveIO extends SubsystemBase {
  
  private Field2d field;
  private RealSwerve realSwerve;
  public SimSwerve simSwerve;
  
  public SwerveIO() {
   
    switch(Constants.currentMode){
      case REAL: 
        realSwerve = new RealSwerve();
        simSwerve = null;
        AutoBuilder.configureHolonomic(
            realSwerve::getPose, // Robot pose supplier
            realSwerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            realSwerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            realSwerve::driveRobotRelitive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
            this // Reference to this subsystem to set requirements
      );
      case SIM: 
        simSwerve=new SimSwerve();
        realSwerve=null;
        AutoBuilder.configureHolonomic(
            simSwerve::getSimPose, // Robot pose supplier
            simSwerve::resetSimOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            simSwerve::getSimSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            simSwerve::driveSimRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
    }
    field = new Field2d();
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    switch(Constants.currentMode){
      case REAL: 
        realSwerve.swerveOdometry.update(realSwerve.getYaw(), realSwerve.getPositions());
        field.setRobotPose(realSwerve.getPose());
        SmartDashboard.putNumber("gyro yaw",  realSwerve.getYaw().getDegrees());
        SmartDashboard.putNumber("robot pose x", realSwerve.getPose().getX());
        SmartDashboard.putNumber("robot pose y", realSwerve.getPose().getY());
        for (SwerveModule mod : realSwerve.mSwerveMods) {
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Integrated Angle", mod.getState().angle.getDegrees());
          SmartDashboard.putNumber(
              "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
      case SIM:
      simSwerve.Simgyro.updateRotation(simSwerve.getSimSpeeds().omegaRadiansPerSecond);
      simSwerve.Simodometry.update(simSwerve.Simgyro.getRotation2d(), simSwerve.getSimPositions());
      field.setRobotPose(simSwerve.getSimPose());
    }
  }

  public void resetOdometry(Pose2d pose2d){
    switch(Constants.currentMode){
      case REAL: realSwerve.resetOdometry(pose2d);break;
      case SIM: simSwerve.resetSimOdometry(pose2d);break;
    }
  }
  public Pose2d getPose(){
    switch(Constants.currentMode){
      case REAL: return realSwerve.getPose();
      case SIM: return simSwerve.getSimPose();
      default: return new Pose2d();
    }
  }
  public Rotation2d getYaw(){
    switch(Constants.currentMode){
      case REAL: return realSwerve.getYaw();
      case SIM: return simSwerve.Simgyro.getRotation2d();
      default: return new Rotation2d();
    }
  }
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    switch(Constants.currentMode){
      case REAL: realSwerve.drive(translation, rotation, fieldRelative, isOpenLoop);
      case SIM: simSwerve.driveSim(translation, rotation, fieldRelative, isOpenLoop);
    }
  }
  public void ResetWheels(){
    if(RobotBase.isReal()){
      realSwerve.setWheelsToX();
    }
  }
  public void zeroGyro(){
    if(RobotBase.isReal()){
      realSwerve.zeroGyro();
    }
  }
  public void zeroOdometry(){
    if(RobotBase.isSimulation()){
      simSwerve.zeroOdometry();
    }
  }
}
