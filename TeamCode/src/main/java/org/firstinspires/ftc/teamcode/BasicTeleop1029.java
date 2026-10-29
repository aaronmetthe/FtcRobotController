package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;               // PINPOINT
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;    // PINPOINT
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; // PINPOINT
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;       // PINPOINT

@TeleOp(name = "BasicTeleop 10-29 (Pinpoint)", group = "BioBuzz")
public class BasicTeleop1029 extends LinearOpMode {   // must match the file name exactly

    private DcMotor back_left_motor;
    private DcMotor back_right_motor;
    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor top_fly_wheelleft;
    private DcMotor tilt_motor_1;
    private CRServo launch_servo_0;
    private CRServo launch_servo_1;
    private GoBildaPinpointDriver pinpoint;                                // PINPOINT

    @Override
    public void runOpMode() {
        ElapsedTime launchTimer = new ElapsedTime();
        boolean isLaunching = false;
        int flipState = 0;
        float true_y_stick;
        float front_right;
        float front_left;
        float back_right;
        float back_left;

        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        top_fly_wheelleft = hardwareMap.get(DcMotor.class, "top_fly_wheelleft");
        tilt_motor_1 = hardwareMap.get(DcMotor.class, "tilt_motor_1");
        launch_servo_0 = hardwareMap.get(CRServo.class, "launch_servo_0");
        launch_servo_1 = hardwareMap.get(CRServo.class, "launch_servo_1");

        // ---------- PINPOINT SETUP ----------
        // Use the same settings that worked in PinpointTest.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);   // update after measuring pods on the robot
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();                    // robot must be sitting still during Init
        // ------------------------------------

        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        top_fly_wheelleft.setDirection(DcMotor.Direction.FORWARD);
        tilt_motor_1.setDirection(DcMotor.Direction.FORWARD);
        launch_servo_0.setDirection(CRServo.Direction.FORWARD);
        launch_servo_1.setDirection(CRServo.Direction.REVERSE);

        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) top_fly_wheelleft).setVelocityPIDFCoefficients(300, 0, 0, 50);

        telemetry.addLine("Ready - press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();                        // PINPOINT: get fresh position every loop

            // ---------- DRIVE ----------
            true_y_stick = -1 * gamepad1.left_stick_y;
            front_right = true_y_stick - (gamepad1.left_stick_x + gamepad1.right_stick_x);
            front_left  = true_y_stick + gamepad1.left_stick_x + gamepad1.right_stick_x;
            back_right  = true_y_stick + (gamepad1.left_stick_x - gamepad1.right_stick_x);
            back_left   = true_y_stick - (gamepad1.left_stick_x - gamepad1.right_stick_x);
            if (gamepad1.left_bumper) {
                back_left /= 3;  back_right /= 3;  front_left /= 3;  front_right /= 3;
            }
            if (gamepad1.right_bumper) {
                back_left /= 6;  back_right /= 6;  front_left /= 6;  front_right /= 6;
            }
            front_right_motor.setPower(front_right);
            front_left_motor.setPower(front_left);
            back_right_motor.setPower(back_right);
            back_left_motor.setPower(back_left);

            // ---------- TILT ----------
            // FIXED: was "(flipState = 0) == 0", which SET flipState to 0 instead of checking it.
            if (gamepad1.dpad_down && flipState == 0) {
                tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tilt_motor_1.setTargetPosition(21900);
                tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tilt_motor_1.setPower(1);
                flipState = 1;
            }
            if (gamepad1.dpad_up && flipState == 0) {
                tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tilt_motor_1.setTargetPosition(-2250);
                tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tilt_motor_1.setPower(1);
                flipState = 1;
            }
            if (gamepad1.dpad_right && flipState == 0) {
                tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tilt_motor_1.setTargetPosition(-3000);
                tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tilt_motor_1.setPower(1);
                flipState = 1;
            }
            // FIXED: was "flipState == 2", which never happened, so the tilt never unlocked.
            if (flipState == 1 && !tilt_motor_1.isBusy()) {
                flipState = 0;
            }

            // ---------- LAUNCHER ----------
            if (gamepad2.left_bumper && !isLaunching) {
                launch_servo_0.setPower(0.285);
                launch_servo_1.setPower(0.285);
                launchTimer.reset();
                isLaunching = true;
            }
            if (isLaunching && launchTimer.milliseconds() > 2400) {
                launch_servo_0.setPower(0);
                launch_servo_1.setPower(0);
                isLaunching = false;
            }
            if (gamepad2.dpad_down) {
                ((DcMotorEx) top_fly_wheelleft).setVelocity(1600);
            }
            if (gamepad2.dpad_up) {
                ((DcMotorEx) top_fly_wheelleft).setVelocity(1880);
            }

            // ---------- PINPOINT RESET ----------
            // Press BACK on gamepad 1 to zero position. Only while the robot is still.
            if (gamepad1.back) {
                pinpoint.resetPosAndIMU();
            }

            // ---------- TELEMETRY ----------
            Pose2D pos = pinpoint.getPosition();      // PINPOINT
            telemetry.addData("X (in)", "%.1f", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.1f", pos.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Tilt pos", tilt_motor_1.getCurrentPosition());
            telemetry.addData("Flywheel vel", ((DcMotorEx) top_fly_wheelleft).getVelocity());
            telemetry.addData("Launching", isLaunching);
            telemetry.update();
        }
    }
}
