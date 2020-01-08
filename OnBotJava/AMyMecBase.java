package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is the base class for my mecanum based robot. It tunes parameters from AMecBase for
 * my robot and includes the programming for motors, servos, sensors added to my robopt so
 * it can play the game. All of my opmodes extend this common base so I can have operations
 * like grabFoundation() or releaseFoundation() that can be used by allo of my teleop and
 * autonomous programs so if I tune the mechanism a bit, I adjust the programming here
 * (in just one place) and all of my opmodes inherit those changes.
 */
public abstract class AMyMecBase extends AMecBase {

    // -------------------------------------------------------------------------------------
    // Add common variables for my robot here. These would be motors, servos, sensors,
    // 'constants' for control of motors, servos, sensors, etc.
    // -------------------------------------------------------------------------------------

    protected void initConstants() {
        super.initConstants();

        // adjust 'constants' initialized in AMecBase that are specific to my robot
        //ticsPerInchForward = 300.0;
        //ticsPerInchSideways = 315.0;
/
        // initialize 'constants' that are specific to the non-drive parts of my
        // robot here.

    }


    protected void preStartInitialization() {
        super.preStartInitialization();

        // Adjust drive motor initialization that is specific to my robot

        // Add initialization for motors, servos, sensors that are the non-drive
        // part of my robot.

    }

    protected void postStartInitialization() {
        super.postStartInitialization();

        // Add any initialization that should happen immediately after start
        // like positioning servos and stuff that may extend outside th 18"
        // robot cube.
    }

    // -------------------------------------------------------------------------------------
    // add your shared (used by multiple op modes0 methods here, things like:
    // * grab and release foundation
    // * sense skystone
    // * sweep block
    // * lift tower
    // * etc.

}
