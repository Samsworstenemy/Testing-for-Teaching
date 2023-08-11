package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class SubsystemPhotonvision extends SubsystemBase{

    /*  :> Sets the name that Photonvision is going to use for the camera to the camera
            that Photonvision itself recognizes */
    public static final String camera1name =  Constants.Photonvision.cameraName1;
    
    // :> Make a variable equal to the position of the Camera on the robot
    public static final Transform3d camToBot = new Transform3d(
        new Pose3d(Units.inchesToMeters(-14), 0, Units.inchesToMeters(22.5),
            new Rotation3d(0, 0, Units.degreesToRadians(180))), new Pose3d());
    
    public static PhotonTrackedTarget targets = new PhotonTrackedTarget();
    public static Pose3d tagPoses = new Pose3d();


    // :> Makes a field object for the field layout
    static AprilTagFieldLayout m_aprilTagFieldLayout;
    static Transform3d cameraToTargets = new Transform3d();

    // :> Makes a camera object based on the camera that Photonvision recognizes
    private PhotonCamera camera = new PhotonCamera(Constants.Photonvision.cameraName1);

    // :> Makes an object for the Position Estimator which estimates the position later
    private PhotonPoseEstimator frontCamPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, camera, camToBot);


    // :> This is the pupose of this subsystem. This gets the robot position when the method is called
    public Pose3d checkRobotPosition() {
        // :> makes sure targets isn't null to prevent null pointers        
        if (targets!= null) {
          
          // :> Makes a estimated pose from the pose estimator
          Optional<EstimatedRobotPose> frontEstPose = frontCamPoseEstimator.update();
          
          // :> Another null pointer prevention if statement
          if(frontEstPose.isPresent()) {
            // :> Returns the estimated pose
            return frontEstPose.get().estimatedPose;
          }

        }
        
        return null;
    }


    public SubsystemPhotonvision() {
        
      /*  :> Okay this seems stupid but Photonvision can't handle an IOException. 
              It can however handle a Runtime Exception
      */

        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException(err);
        }
        
        // :> Sets the field april tags to the actual field layout
        frontCamPoseEstimator.setFieldTags(m_aprilTagFieldLayout);

        // :> If it can't use the strategy this makes it fall back on a different strategy to use.
        frontCamPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }

 
    @Override
  public void periodic() {

    // :> relates the camera results to well, the results from the cameras
    PhotonPipelineResult result = camera.getLatestResult();
    
    /* :> Checks if targets is empty as a precaution to make sure that it isn't getting 
        values from targets that don't exist
    */ 
    if (result.hasTargets() && result.getBestTarget().getPoseAmbiguity() < 0.07) {
      Pose3d robotPose = checkRobotPosition();
     
      // :> Another null pointer prevention if statement
      if (robotPose != null) {
        MotorSubsystem.funnyPose = robotPose.toPose2d();
      }
    }
  }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
    // :> This method checks to see if it can see an apriltag to update Shuffleboard
    public boolean seeingApriltag() {
      return camera.getLatestResult().hasTargets();
    }
  } 
