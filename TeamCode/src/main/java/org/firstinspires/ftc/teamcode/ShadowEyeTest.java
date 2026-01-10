

package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



import java.util.List;

@TeleOp(name="ShadowEyeTest", group="Linear OpMode")
public class ShadowEyeTest extends LinearOpMode {

    /* =======================
       DRIVE / MECHANISMS
       ======================= */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Fleft, Bleft, Fright, Bright;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake1, intake2;

    /* =======================
       APRILTAG / VISION
       ======================= */
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final boolean USE_WEBCAM = true;

    private final Position cameraPosition = new Position(
            DistanceUnit.INCH, 0, 0, 0, 0);

    private final YawPitchRollAngles cameraOrientation =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);


    // ===== Camera image info (match your resolution) =====
    private static final double IMG_CENTER_X = 320;  // 640x480
    private static final double IMG_CENTER_Y = 240;

    // ===== Pixel align tuning =====
    private static final double ALIGN_KP_YAW = 0.0025;
    private static final double ALIGN_MAX_TURN = 0.35;
    private static final double ALIGN_DEADBAND = 10;

    @Override
    public void runOpMode() {

        /* ===== Hardware Init ===== */
        Fleft   = hardwareMap.get(DcMotor.class, "Fleft");
        Bleft   = hardwareMap.get(DcMotor.class, "Bleft");
        Fright  = hardwareMap.get(DcMotor.class, "Fright");
        Bright  = hardwareMap.get(DcMotor.class, "Bright");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake1  = hardwareMap.get(DcMotor.class, "intake1");
        intake2  = hardwareMap.get(DcMotor.class, "intake2");

        Fleft.setDirection(DcMotor.Direction.REVERSE);
        Bleft.setDirection(DcMotor.Direction.REVERSE);
        Fright.setDirection(DcMotor.Direction.FORWARD);
        Bright.setDirection(DcMotor.Direction.FORWARD);

        shooter1.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        shooter1.setVelocityPIDFCoefficients(600, 0, 0, 0);

        /* ===== Vision Init ===== */
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        /* =======================
           MAIN LOOP
           ======================= */
        while (opModeIsActive()) {


            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (gamepad1.right_trigger > 0.3 && detections.size() > 0) {

                // AUTO ALIGN MODE
                alignToTagPixels(detections.get(0));


            } else {

                // ===== NORMAL DRIVER CONTROL =====
                double axial   = -gamepad1.left_stick_y;
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                double fl = axial + lateral + yaw;
                double fr = axial - lateral - yaw;
                double bl = axial - lateral + yaw;
                double br = axial + lateral - yaw;

                double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br)));

                if (max > 1.0) {
                    fl /= max; fr /= max; bl /= max; br /= max;
                }

                Fleft.setPower(fl);
                Fright.setPower(fr);
                Bleft.setPower(bl);
                Bright.setPower(br);
            }


            /* ----- Shooter ----- */
            shooter1.setVelocity(-750);
            shooter2.setPower(shooter1.getPower());

            /* ----- Intake ----- */
            intake2.setPower(gamepad2.a ? 1 :
                    gamepad2.b ? -1 : 0);

            intake1.setPower(gamepad2.left_bumper ? 1 :
                    gamepad2.right_bumper ? -1 : 0);

            /* ----- AprilTag Telemetry ----- */
            telemetryAprilTag();

            telemetry.addData("Run Time", runtime.toString());
            telemetry.update();

            sleep(20);
        }

        visionPortal.close();
    }

    /* =======================
       APRILTAG SETUP
       ======================= */
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "shadoweye"));
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /* =======================
       APRILTAG TELEMETRY
       ======================= */
    private void telemetryAprilTag() {

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags", detections.size());

        for (AprilTagDetection det : detections) {
            if (det.metadata != null) {
                telemetry.addLine(
                        String.format("ID %d (%s)", det.id, det.metadata.name));

                telemetry.addLine(
                        String.format("X %.1f  Y %.1f  Z %.1f (in)",
                                det.robotPose.getPosition().x,
                                det.robotPose.getPosition().y,
                                det.robotPose.getPosition().z));

                telemetry.addLine(
                        String.format("Yaw %.1f°",
                                det.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            }
        }
    }
    private void stopDrive() {
        Fleft.setPower(0);
        Fright.setPower(0);
        Bleft.setPower(0);
        Bright.setPower(0);
    }
    private void alignToTagPixels(AprilTagDetection det) {
        // --- Convert pixel error ---
        double errorX = det.center.x - IMG_CENTER_X;
        double errorY = IMG_CENTER_Y - det.center.y; // positive = move forward

// --- Gains ---
        double kP_yaw = 0.0025;
        double kP_strafe = 0.002;
        double maxTurn = 0.35;
        double maxStrafe = 0.25;

// --- Deadband ---
        if (Math.abs(errorX) < ALIGN_DEADBAND) {
            stopDrive();
            return;
        }

// --- Corrections ---
        double rotate = errorX * kP_yaw;
        double strafe = errorX * kP_strafe;

// --- Clamp ---
        rotate = Math.max(-maxTurn, Math.min(maxTurn, rotate));
        strafe = Math.max(-maxStrafe, Math.min(maxStrafe, strafe));

// --- Apply mecanum ---
        double fl =  strafe + rotate;
        double fr = -strafe - rotate;
        double bl = -strafe + rotate;
        double br =  strafe - rotate;

        Fleft.setPower(fl);
        Fright.setPower(fr);
        Bleft.setPower(bl);
        Bright.setPower(br);

    }
}

