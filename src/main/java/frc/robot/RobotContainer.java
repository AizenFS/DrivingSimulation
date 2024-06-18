// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveIO;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {

  private Command visionAuto(){
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        swerveIO.getPose(),
        new Pose2d(5.8, -2.5, Rotation2d.fromDegrees(0)));

        PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), 
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

     path.preventFlipping =true;
     swerveIO.resetOdometry(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path);
  }
  private Command testAuto(){
       PathPlannerPath path = PathPlannerPath.fromPathFile("Work");
       path.preventFlipping =true;
       return AutoBuilder.followPath(path);
  }

  private Command Auto1(){
    swerveIO.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 1"));
    return AutoBuilder.buildAuto("Auto 1");
  }
   private Command Auto2(){
    swerveIO.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 2"));
    return AutoBuilder.buildAuto("Auto 2");
  }
   private Command Auto3(){
    swerveIO.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 3"));
    return AutoBuilder.buildAuto("Auto 3");
  }
   private Command Auto4(){
    swerveIO.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 4"));
    return AutoBuilder.buildAuto("Auto 4");
  }
   private Command Auto5(){
    swerveIO.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("Auto 5"));
    return AutoBuilder.buildAuto("Auto 5");
  }
  public Command NullAuto(){
    return null;
  }
  private LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto Routine",AutoBuilder.buildAutoChooser());
  private CommandXboxController xboxController;
  private CommandPS5Controller ps5Controller;

  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;
  
  /* Subsystems */
  public static SwerveIO swerveIO = new SwerveIO();
  
  
  public RobotContainer() {
    configureBindings(); 
    configureChooser();
  }

  private void configureBindings(){
    switch(Constants.currentController){
      case PS5:
        ps5Controller=new CommandPS5Controller(0);
        translationAxis=PS5Controller.Axis.kLeftY.value;
        strafeAxis = PS5Controller.Axis.kLeftX.value;
        rotationAxis = PS5Controller.Axis.kRightX.value;
        swerveIO.setDefaultCommand(
          new TeleopSwerve(
              swerveIO,
              () -> -ps5Controller.getRawAxis(translationAxis),
              () -> -ps5Controller.getRawAxis(strafeAxis),
              () -> -ps5Controller.getRawAxis(rotationAxis),
              () -> ps5Controller.circle().getAsBoolean()));

       case XBOX:
        xboxController=new CommandXboxController(0);
        translationAxis=XboxController.Axis.kLeftY.value;
        strafeAxis = XboxController.Axis.kLeftX.value;
        rotationAxis = XboxController.Axis.kRightX.value;
        swerveIO.setDefaultCommand(
          new TeleopSwerve(
            swerveIO,
            () -> -xboxController.getRawAxis(translationAxis),
            () -> -xboxController.getRawAxis(strafeAxis),
            () -> - xboxController.getRawAxis(rotationAxis),
            () -> xboxController.y().getAsBoolean()
          )
        );
        xboxController.a().onTrue(new InstantCommand(() -> swerveIO.zeroOdometry()));
    }
  }
  private void configureChooser() {
    
    
    chooser.addDefaultOption("no auto", NullAuto());
    chooser.addOption("Auto 1", Auto1());
    chooser.addOption("Auto 2", Auto2());
    chooser.addOption("Auto 3", Auto3());
    chooser.addOption("Auto 4", Auto4());
    chooser.addOption("Auto 5", Auto5());
    chooser.addOption("vision auto", visionAuto());
    chooser.addOption("test", testAuto());
    SmartDashboard.putData("test",AutoBuilder.followPath(PathPlannerPath.fromPathFile(("Left Ring"))));

  }

 
  public Command getAutonomousCommand() {

    return chooser.get();
  }
}
