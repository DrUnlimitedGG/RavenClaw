

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    public DcMotor RF = hardwareMap.get(DcMotor.class, "right_front");
    public DcMotor LF = hardwareMap.get(DcMotor.class, "left_front");
    public DcMotor RB = hardwareMap.get(DcMotor.class, "right_back");
    public DcMotor LB = hardwareMap.get(DcMotor.class, "left_back");

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        LF.setPower(ly + lx + rx);
        LB.setPower(ly - lx + rx);
        RF.setPower(ly - lx - rx);
        RB.setPower(ly + lx - rx);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}



