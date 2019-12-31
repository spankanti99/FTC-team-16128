package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

@Autonomous(name = "autoskystonedetection (Blocks to Java)", group = "")
public class autoskystonedetection extends LinearOpMode {

  private DcMotor frontright;
  private DcMotor frontleft;
  private DcMotor backright;
  private DcMotor backleft;
  private DcMotor m2;
  private VuforiaSkyStone vuforiaSkyStone;
  private TfodSkyStone tfodSkyStone;
  private DcMotor m3;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontright = hardwareMap.dcMotor.get("frontright");
    frontleft = hardwareMap.dcMotor.get("frontleft");
    backright = hardwareMap.dcMotor.get("backright");
    backleft = hardwareMap.dcMotor.get("backleft");
    m2 = hardwareMap.dcMotor.get("m2");
    vuforiaSkyStone = new VuforiaSkyStone();
    tfodSkyStone = new TfodSkyStone();
    m3 = hardwareMap.dcMotor.get("m3");

    // Initialization
    frontright.setPower(0);
    frontleft.setPower(0);
    backright.setPower(0);
    backleft.setPower(0);
    telemetry.addData("Init ", "started");
    telemetry.update();
    frontright.setPower(0);
    frontleft.setPower(0);
    backright.setPower(0);
    backleft.setPower(0);
    frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    backright.setDirection(DcMotorSimple.Direction.REVERSE);
    m2.setTargetPosition(1.5);
    // Init Vuforia because Tensor Flow needs it.
    vuforiaSkyStone.initialize(
        "", // vuforiaLicenseKey
        VuforiaLocalizer.CameraDirection.BACK, // cameraDirection
        true, // useExtendedTracking
        true, // enableCameraMonitoring
        VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
        0, // dx
        0, // dy
        0, // dz
        0, // xAngle
        0, // yAngle
        0, // zAngle
        false); // useCompetitionFieldTargetLocations
    telemetry.addData("Vuforia", "initialized");
    telemetry.update();
    // Let's use 70% minimum confidence and
    // and no object tracker.
    tfodSkyStone.initialize(vuforiaSkyStone, 0.7F, true, true);
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    // Set target ratio of object height to image
    // height value corresponding to the length
    // of the robot's neck.
    TargetHeightRatio = 0.68;
    waitForStart();
    tfodSkyStone.activate();
    // We'll loop until gold block captured or time is up
    SkystoneFound = false;
    while (opModeIsActive() && !SkystoneFound) {
      // Get list of current recognitions.
      recognitions = tfodSkyStone.getRecognitions();
      // Report number of recognitions.
      telemetry.addData("Objects Recognized", recognitions.size());
      // If some objects detected...
      if (recognitions.size() > 0) {
        // ...let's count how many are gold.
        SkystoneCount = 0;
        // Step through the stones detected.
        // TODO: Enter the type for variable named recognition
        for (UNKNOWN_TYPE recognition : recognitions) {
          if (recognition.getLabel().equals("Skystone")) {
            // A Skystone has been detected.
            SkystoneCount = SkystoneCount + 1;
            // We can assume this is the first Skystone
            // because we break out of this loop below after
            // using the information from the first Skystone.
            // We don't need to calculate turn angle to Skystone
            // because TensorFlow has estimated it for us.
            ObjectAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
            // Negative angle means Skystone is left, else right.
            telemetry.addData("Estimated Angle", ObjectAngle);
            if (ObjectAngle < -6) {
              frontleft.setPower(-0.2);
              backleft.setPower(0.2);
              frontright.setPower(0.2);
              backright.setPower(-0.2);
              telemetry.addData("Direction", "left");
            } else if (ObjectAngle > 6) {
              telemetry.addData("Direction", "right");
              frontleft.setPower(0.2);
              backleft.setPower(-0.2);
              frontright.setPower(-0.2);
              backright.setPower(0.2);
            } else {
              ImageHeight = recognition.getImageHeight();
              ObjectHeight = recognition.getHeight();
              //
              // Calculate height of Skystone relative to image height.
              // Larger ratio means robot is closer to Skystone.
              ObjectHeightRatio = ObjectHeight / ImageHeight;
              telemetry.addData("HeightRatio", ObjectHeightRatio);
              // Use height ratio to determine distance.
              // If height ratio larger than (target - tolerance)...
              LeftPower = 0.08 + 0.5 * ((TargetHeightRatio - 0.05) - ObjectHeightRatio);
              RightPower = LeftPower;
              if (ObjectHeightRatio < TargetHeightRatio - 0.03) {
                // ...not close enough yet.
                telemetry.addData("Distance", "Not close enough");
                // If sum of turn powers are small
                if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.2) {
                  // ...don't really need to turn.  Move forward.
                  telemetry.addData("Action", "Forward");
                  // Go forward by setting power proportional to how
                  // far from target distance.
                  LeftPower = 0.08 + 0.5 * ((TargetHeightRatio - 0.05) - ObjectHeightRatio);
                  RightPower = LeftPower;
                } else {
                  // Else we'll turn to Skystone with current power levels.
                  telemetry.addData("Action", "Turn");
                }
                // Else if height ratio more than (target+tolerance)...
              } else if (ObjectHeightRatio > TargetHeightRatio + 0.05) {
                // ...robot too close to Skystone.
                telemetry.addData("Distance", "Too close");
                // If calculated turn power levels are small...
                if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.2) {
                  // ...don't need to turn.  Backup instead by setting
                  // power proportional to how far past target ratio
                  telemetry.addData("Action", "Back up");
                  LeftPower = -0.08 + -0.5 * ((TargetHeightRatio + 0.05) - TargetHeightRatio);
                  RightPower = LeftPower;
                } else {
                  // Else use current power levels to turn to Skystone
                  telemetry.addData("Action", "Turn");
                }
              } else {
                // Skystone is about one neck length away.
                telemetry.addData("Distance", "Correct");
                // If calculated turn power levels are small...
                if (Math.abs(LeftPower) + Math.abs(RightPower) < 0.12) {
                  // ...robot is centered on the Skystone.
                  telemetry.addData("Action", "Motors off, hit the Skystone");
                  // move close to Skystone to pick up
                  // Lower neck and open jaw.
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count = 0; count < 12; count++) {
                    m2.setPower(-0.5);
                  }
                  m2.setPower(-0.75);
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count2 = 0; count2 < 40; count2++) {
                    frontright.setPower(0.3);
                    backright.setPower(0.3);
                    frontleft.setPower(0.3);
                    backleft.setPower(0.3);
                  }
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count3 = 0; count3 < 15; count3++) {
                    frontright.setPower(-0.5);
                    backright.setPower(0.5);
                    frontleft.setPower(0.5);
                    backleft.setPower(-0.5);
                  }
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count4 = 0; count4 < 20; count4++) {
                    m3.setPower(-0.5);
                  }
                  sleep(500);
                  for (int count5 = 0; count5 < 75; count5++) {
                    frontright.setPower(-0.5);
                    backright.setPower(-0.5);
                    frontleft.setPower(-0.5);
                    backleft.setPower(-0.5);
                  }
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count6 = 0; count6 < 200; count6++) {
                    frontright.setPower(0.5);
                    backright.setPower(-0.5);
                    frontleft.setPower(-0.5);
                    backleft.setPower(0.5);
                  }
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  backright.setPower(0);
                  backleft.setPower(0);
                  for (int count7 = 0; count7 < 15; count7++) {
                    m2.setPower(1);
                  }
                  sleep(500);
                  for (int count8 = 0; count8 < 100; count8++) {
                    frontright.setPower(0.3);
                    backright.setPower(0.3);
                    frontleft.setPower(0.3);
                    backleft.setPower(0.3);
                  }
                  backright.setPower(0);
                  backleft.setPower(0);
                  frontleft.setPower(0);
                  frontright.setPower(0);
                  for (int count9 = 0; count9 < 15; count9++) {
                    m2.setPower(-1);
                  }
                  for (int count10 = 0; count10 < 50; count10++) {
                    frontright.setPower(-0.3);
                    backright.setPower(-0.3);
                    frontleft.setPower(-0.3);
                    backleft.setPower(-0.3);
                  }
                  for (int count11 = 0; count11 < 12; count11++) {
                    m3.setPower(0.3);
                  }
                  SkystoneFound = true;
                } else {
                  // Otherwise use current power levels to turn
                  // to better center on gold.
                  telemetry.addData("Action", "Turn");
                }
              }
              RightPower = 0.3;
              LeftPower = 0.3;
              telemetry.addData("Left Power", LeftPower);
              telemetry.addData("Right Power", RightPower);
              // Set power levels to get closer to Skystone.
              frontleft.setPower(LeftPower);
              frontright.setPower(RightPower);
              backleft.setPower(LeftPower);
              backright.setPower(RightPower);
              // We've found a Skystone so we don't have
              // to look at rest of detected objects.
              // Break out of For-each-recognition.
              break;
            }
            // Calculate power levels for turn toward Skystone.
            // We'll be comparing the Skystone height
            // to the height of the video image to estimate
          }
        }
        // If no Skystones detected...
        if (SkystoneCount == 0) {
          telemetry.addData("Status", "No Skystone");
          telemetry.addData("Action", "Back up");
          // Back up slowly hoping to bring Skystone in view.
          frontleft.setPower(0.2);
          frontright.setPower(-0.2);
          backleft.setPower(-0.2);
          backright.setPower(0.2);
        }
      } else {
        // No objects detected
        telemetry.addData("Status", "No objects detected");
        telemetry.addData("Action", "Back up");
        // Back up slowly hoping to bring objects in view.
        frontleft.setPower(-0.2);
        frontright.setPower(-0.2);
        backleft.setPower(-0.2);
        backright.setPower(-0.2);
      }
      telemetry.update();
    }
    // Skystone found, time is up or stop was requested.
    tfodSkyStone.deactivate();
    frontleft.setPower(0);
    frontright.setPower(0);
    backleft.setPower(0);
    backright.setPower(0);
    // Pause to let driver station to see last telemetry.
    sleep(2000);

    vuforiaSkyStone.close();
    tfodSkyStone.close();
  }
}