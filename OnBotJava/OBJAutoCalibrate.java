package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The OnBotJava version of the blocks AutoCalibrate.
 */
@TeleOp(name = "OBJAutoCalibrate", group = "")
public class OBJAutoCalibrate extends AMyMecBase {
    double calibrationDistance;

    protected void initConstants() {
        super.initConstants();
        calibrationDistance = 24.0;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // initialize the control constants
        initConstants();
        // initialize the hardware before the opmode starts
        preStartInitialization();
        waitForStart();
        // the OpMode has started, do any initialization that needs to be postponed until the OpMode starts.
        postStartInitialization();
        // OK, the robot is ready to go
        // This is the main run loop - we are just waiting for input that initiates autonomous move commands
        while (opModeIsActive()) {
            // Gamepad X is the forced exit from the program (only works when the robot is not executing a move/rotate).
            if (gamepad1.x) {
                break;
            }
            // Set the max power for the move/rotate based on the right bumper.
            double maxPower = gamepad1.right_bumper ? 0.5 : 1.0;
            // The DPAD initiates the autonomous moves for calibration.
            if (gamepad1.dpad_up) {
                // Move forward the calibration distance
                move(calibrationDistance, 0.0, maxPower);
            } else if (gamepad1.dpad_down) {
                // Move backwards the calibration distance
                move(-calibrationDistance, 0.0, maxPower);
            } else if (gamepad1.dpad_right) {
                if (gamepad1.left_bumper) {
                    // Rotate 90 clockwise
                    rotate(90.0, maxPower);
                } else {
                    // Move right the calibration distance
                    move(calibrationDistance, 90.0, maxPower);
                }
            } else if (gamepad1.dpad_left) {
                if (gamepad1.left_bumper) {
                    // Rotate 90 counter-clockwise
                    rotate(-90.0, maxPower);
                } else {
                    // Move left the calibration distance
                    move(calibrationDistance, -90.0, maxPower);
                }
            }
        }
    }

}
