package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.stream.DoubleStream;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.LimeLightHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class LimeLight extends SubsystemBase {
    

  //create NetworkTable objects
  NetworkTableInstance nInstance;
  NetworkTable table; 

  
  
  //limelight values
  private NetworkTableEntry ta; 
  private NetworkTableEntry tv; 
  private NetworkTableEntry ty;
  private NetworkTableEntry tx; 
  private NetworkTableEntry tid; 
  private double[] tagpose, botpose; 
  private int pipeline; 

  //trajectory fields
  private Trajectory trajectory;

  HashMap<Double, Pose2d> poses; 
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();


  public LimeLight() 
  {
      System.out.println("Limelight object initialized");

      nInstance = NetworkTableInstance.getDefault();
      table = nInstance.getTable("limelight");
      ta = table.getEntry("ta");
      tv = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      tid = table.getEntry("tid");
      tagpose = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); 
      botpose = table.getEntry("robotpose_targetspace").getDoubleArray(new double[6]); 
      poses = new HashMap<Double, Pose2d>();
      
      
  
  }

  public Pose2d getTagPose() { 
      return LimeLightHelpers.getBotPose2d("limelight");
  }


  public void printTargetPoses() {
      System.out.println(Arrays.asList(poses));
  }

  public double getTa() {
      return ta.getDouble(0); 
  }

  public double getTv() {
      return tv.getDouble(0); 
  }

  public double getTx() {
      return tx.getDouble(0); 
  }

  public double getTy() {
      return ty.getDouble(0); 
  }

  public double getTid() {
      return tid.getDouble(0); 
  }

  public String getPipeline() {
      return "Currently using pipeline " + pipeline; 
  }

  public void setPipeline(int pipeline) {
      this.pipeline = pipeline;
      table.getEntry("pipeline").setNumber(pipeline);
      System.out.println("Pipeline changed to " + pipeline); 
  }
 
  public Pose3d[] getDetectedAprilTagsPoses(){
    LimeLightHelpers.LimelightResults llresults = LimeLightHelpers.getLatestResults("limelight");
    LimeLightHelpers.LimelightTarget_Fiducial[] aprilTags = llresults.targetingResults.targets_Fiducials;
    Pose3d[] tagPoseArray = new Pose3d[aprilTags.length];
    for (int i = 0; i < aprilTags.length; i++) {
        tagPoseArray[i] = aprilTags[i].getTargetPose_RobotSpace();//TODO might change
    }
    return tagPoseArray;
  }
  public int[] getDetectedAprilTagsId(){
    double[] aprilTagIds = tid.getDoubleArray(new double[0]);
    int[] intIds = DoubleStream.of(aprilTagIds).mapToInt(d -> (int) Math.ceil(d)).toArray();
    return intIds;
  }

  public Pose3d[] DetectedAprilTagsPosesFromId(int[] ids){
    Pose3d[] tagPose3ds = new Pose3d[ids.length];
    for(int i=0;i<ids.length;i++){
        tagPose3ds[i]=aprilTagFieldLayout.getTagPose(ids[i]).get();
    }
    return tagPose3ds;
  }

  @Override
  public void periodic() {
    if(RobotBase.isReal()){
        SmartDashboard.putNumber("x value", getTagPose().getX());
        SmartDashboard.putNumber("y value", getTagPose().getY());
        Logger.recordOutput("DetectedAprilTags", DetectedAprilTagsPosesFromId(getDetectedAprilTagsId())); //TODO try the other one
    }
  }
  public void mapOriginPairs() {
      //initialize dictionary 
      if (this.getTv() == 1) {
          poses.put(this.getTid(), this.getTagPose());
          System.out.println("Successfully mapped");
      }

      else {
          System.out.println("No mapping to be done");
      }

  }

 


}
