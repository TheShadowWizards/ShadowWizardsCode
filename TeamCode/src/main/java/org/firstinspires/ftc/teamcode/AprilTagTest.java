package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AprilTagTest", group="Test")
public class AprilTagTest extends LinearOpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // ---- APRILTAG PROCESSOR ----
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // ---- VISION PORTAL ----
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "shadoweye"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("VisionPortal initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            telemetry.addData("Tags detected", detections.size());

            for (AprilTagDetection tag : detections) {
                if (tag.ftcPose != null) {
                    telemetry.addData("ID", tag.id);
                    telemetry.addData("Distance (cm)", "%.1f", tag.ftcPose.range);
                    telemetry.addData("Angle (deg)", "%.1f", tag.ftcPose.bearing);
                    telemetry.addLine("--------------------");
                } else {
                    telemetry.addData("ID", tag.id);
                    telemetry.addLine("Pose Unavailable");
                }
            }

            telemetry.update();
        }

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
