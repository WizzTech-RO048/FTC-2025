package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

import static android.os.SystemClock.sleep;

@TeleOp(name = "FTC2025")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Controller controller1, controller2;
    private SampleMecanumDrive drive;

    private int raise_value, arm_value = 0, lift_value;
    public double RAISE_POWER = 1.0;

    public int ARM_MAX_POS = 600;
    public int SLIDER_BASKET_POS = 2700;
    public int SLIDER_CHAMBER_POS = 1500;

    public double arm_percentage = 0.0;      // procent din cat sa ridice din ARM_MAX_POS (are valoarea intre 0.0 si 1.0)
    public double slider_percentage = 0.0;   // procent din cat sa ridice din SLIDER_MAX_POS (are valoarea intre 0.0 si 1.0)

    private boolean closed, armIsUp;
    private int gripper_position = 0; //0-oprit 1-aduna piesa 2-beleste piesa
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;
    //boolean isPressed = false;

    int slider_min = 0, slider_max = 0;
    int slider_target_position = 0;
    int arm_target_positionup = 0;
    int arm_target_positiondown = 0;

    int arm_target_position = 0;

    boolean isExtended = false;
    boolean gripper_rotating = false;
    boolean lb_down = false;
    boolean lift_position = false;

    boolean gripper_grab = true;

    double movement_speed = 1.0;

    // ----- for generating telemetry logs -----
    private FileWriter writer;
    private long startTime;

    //test ca nu mere movementul
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );

        // ====================================================
        // === controller1 - movement si control intake     ===
        // === controller1 - control slider si control arm  ===
        // ====================================================
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftRear");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightRear");

        last_arm_position = 0;


        // ----- for generating telemetry logs -----
//        File logFile = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/encoder_log.txt");
//        try {
//            writer = new FileWriter(logFile, false); // Overwrites existing file
//            writer.write("Time(ms), Encoder Position\n"); // Header row
//            telemetry.addData("Log", "Initialized. File: encoder_log.txt");
//        } catch (IOException e) {
//            telemetry.addData("Error", "File write failed: " + e.getMessage());
//        }
//        startTime = System.currentTimeMillis();


    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();

        // controller 1
        // - movement
        // - ridcare arm
        // - intake gripper
        // - control slider
        // - lift

        // =======================
        // ===== DRIVER 1 ========
        // =======================

        // --------- movement general al robotului ---------
//        drive.setWeightedDrivePower(
//                new Pose2d( /// imi pare rau pt ce urmeaza sa fac, timpuri disperate...
//                        (-controller1.left_stick_y * movement_speed),
//                        (-controller1.left_stick_x * movement_speed),
//                        (-controller1.right_stick_x * movement_speed)  // gen astea negative / pozitive sau schimbate intre ele
//                )
//        );

        double y = -controller1.left_stick_y;  // Forward is negative in FTC
        double x = controller1.left_stick_x;  // Strafing (slight boost to compensate)
        double rx = -controller1.right_stick_x;  // Rotation

        double frontLeftPower = y + x + rx;
        double backLeftPower = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower = y + x - rx;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        //----------- gripper ---------------

        if (!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return;
        }

        if (controller1.leftBumperOnce()) {
            robot.gripper.outtake_release_position();
            robot.extindere_slider_orizontal();
            gripper_grab = false;
            isExtended = true;

        }
        if (controller1.rightBumperOnce()) {
            robot.retragere_slider_orizontal();
            robot.gripper.outtake_grab_position();
            isExtended = false;
            gripper_grab = true;

        }
//        if (controller1.dpadUpOnce()) {
//            robot.gripper.pass_object_pickup_position();
//        }
        if (controller1.dpadDownOnce()) {
//            robot.gripper.pass_object_release_position();
            robot.slider.raiseSlider(0, 1);

        }
        if (controller1.dpadLeftOnce()) {
            robot.slider.raiseSlider(SLIDER_BASKET_POS, 1);
        }

        if (controller1.dpadRightOnce()) {
            robot.slider.raiseSlider(SLIDER_CHAMBER_POS, 1);
        }

        if (controller1.YOnce()) {
            robot.gripper.score_object_pickup_position();
        }
        if (controller1.XOnce()) {
            robot.gripper.score_object_release_position();
        }
        if (controller1.AOnce()) {
            robot.gripper.score_object_chamber_position();
        }
        if (controller1.BOnce()) {
            if(gripper_grab) {
                robot.gripper.outtake_release_position();
                gripper_grab = !gripper_grab;
            } else {
                robot.gripper.outtake_grab_position();
                gripper_grab = !gripper_grab;
            }
        }

//        -------------slider-------------

        //if(controller1.)
        // ------- printing the slider position -------
        telemetry.addData("SliderLeft position", robot.slider.getCurrentPositionSliderLeft());
        telemetry.addData("SliderRight position", robot.slider.getCurrentPositionSliderRight());
        telemetry.addLine("---------------------");
        //telemetry.addData("Arm position", robot.arm.getCurrentPositionArm());
        //telemetry.addData("Arm position2", robot.arm2.getCurrentPositionArm());
        telemetry.addLine("---------------------");
        telemetry.addLine("---------------------");
        telemetry.addLine("---------------------");
        telemetry.addLine("---------------------");

        // ------ printing data in the telemetry logs file ------
//        long timestamp = System.currentTimeMillis() - startTime;
//        try {
//            if (writer != null) {
//                //writer.write(timestamp + "," + robot.arm.getCurrentPositionArm() + "\n");
//                writer.flush(); // Ensure immediate writing
//            }
//        } catch (IOException e) {
//            telemetry.addData("Error", "Logging failed: " + e.getMessage());
//        }

        // Show telemetry
        //telemetry.addData("Encoder", robot.arm.getCurrentPositionArm());
//        telemetry.addData("Log", "Saving to encoder_log.txt");
//        telemetry.update();


    }
}