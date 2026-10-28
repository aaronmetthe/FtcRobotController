package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleopregionals (Blocks to Java)")
public class BasicTeleop extends LinearOpMode {

    private DcMotor back_left_motor;
    private DcMotor back_right_motor;
    private DcMotor front_left_motor;
    private DcMotor front_right_motor;
    private DcMotor top_fly_wheelleft;
    private DcMotor tilt_motor_1;
    private CRServo launch_servo_0;
    private CRServo launch_servo_1;

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        boolean flywheelOn;
        boolean flywheelon2;
        boolean lastA;
        ElapsedTime launchTimer;
        boolean isLaunching;
        int forwardPower;
        int strafePower;
        int rotatePower;
        int maxWheelPower;
        int frontLeftPower;
        int frontRightPower;
        int backLeftPower;
        int backRightPower;
        boolean slowMode;
        int flipState;
        float true_y_stick;
        float front_right;
        float front_left;
        float back_right;
        float back_left;
        boolean lastdpadup;
        boolean lastdpaddown;

        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        top_fly_wheelleft = hardwareMap.get(DcMotor.class, "top_fly_wheelleftAsDcMotor");
        tilt_motor_1 = hardwareMap.get(DcMotor.class, "tilt_motor_1AsDcMotor");
        launch_servo_0 = hardwareMap.get(CRServo.class, "launch_servo_0");
        launch_servo_1 = hardwareMap.get(CRServo.class, "launch_servo_1");

        // Put initialization blocks here.
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        top_fly_wheelleft.setDirection(DcMotor.Direction.FORWARD);
        tilt_motor_1.setDirection(DcMotor.Direction.FORWARD);
        launch_servo_0.setDirection(CRServo.Direction.FORWARD);
        launch_servo_1.setDirection(CRServo.Direction.REVERSE);
        flywheelOn = false;
        flywheelon2 = false;
        lastA = false;
        launchTimer = new ElapsedTime();
        isLaunching = false;
        forwardPower = 0;
        strafePower = 0;
        rotatePower = 0;
        maxWheelPower = 0;
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
        slowMode = false;
        flipState = 0;
        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder wires, you should remove these
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) top_fly_wheelleft).setVelocityPIDFCoefficients(300, 0, 0, 50);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                true_y_stick = -1 * gamepad1.left_stick_y;
                // Put loop blocks here.
                telemetry.addData("Left Stick X", gamepad1.left_stick_x);
                telemetry.addData("Left Stick Y", true_y_stick);
                telemetry.addData("Right Stick X", gamepad1.right_stick_x);
                telemetry.addData("launcher_servo", gamepad2.left_bumper);
                front_right = true_y_stick - (gamepad1.left_stick_x + gamepad1.right_stick_x);
                front_left = true_y_stick + gamepad1.left_stick_x + gamepad1.right_stick_x;
                back_right = true_y_stick + (gamepad1.left_stick_x - gamepad1.right_stick_x);
                back_left = true_y_stick - (gamepad1.left_stick_x - gamepad1.right_stick_x);
                if (gamepad1.left_bumper) {
                    back_left = back_left / 3;
                    back_right = back_right / 3;
                    front_left = front_left / 3;
                    front_right = front_right / 3;
                }
                if (gamepad1.right_bumper) {
                    back_left = back_left / 6;
                    back_right = back_right / 6;
                    front_left = front_left / 6;
                    front_right = front_right / 6;
                }
                front_right_motor.setPower(front_right);
                front_left_motor.setPower(front_left);
                back_right_motor.setPower(back_right);
                back_left_motor.setPower(back_left);
                if (gamepad1.dpad_down && (flipState = 0) == 0) {
                    tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tilt_motor_1.setTargetPosition(21900);
                    tilt_motor_1.setPower(1);
                    tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    flipState = 1;
                }
                if (gamepad1.dpad_up && (flipState = 0) == 0) {
                    tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tilt_motor_1.setTargetPosition(tilt_motor_1.getCurrentPosition() - 2250);
                    tilt_motor_1.setPower(1);
                    tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    flipState = 1;
                }
                if (gamepad1.dpad_right && (flipState = 0) == 0) {
                    tilt_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    tilt_motor_1.setTargetPosition(tilt_motor_1.getCurrentPosition() - 3000);
                    tilt_motor_1.setPower(1);
                    tilt_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    flipState = 1;
                }
                if (flipState == 2 && !tilt_motor_1.isBusy()) {
                    flipState = 0;
                }
                if (gamepad2.left_bumper && !isLaunching) {
                    launch_servo_0.setPower(0.285);
                    launch_servo_1.setPower(0.285);
                    launchTimer.reset();
                    isLaunching = true;
                }
                if (gamepad2.dpad_down) {
                    ((DcMotorEx) top_fly_wheelleft).setVelocity(1600);
                }
                if (gamepad2.dpad_up) {
                    ((DcMotorEx) top_fly_wheelleft).setVelocity(1880);
                }
                if (isLaunching) {
                    if (launchTimer.milliseconds() > 2400) {
                        launch_servo_0.setPower(0);
                        launch_servo_1.setPower(0);
                        isLaunching = false;
                    }
                }
                telemetry.update();
            }
        }
    }
}
