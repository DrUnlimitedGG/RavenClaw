
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="CarouselTest", group="Testing Stuff")
@Disabled
public class CarouselTest extends OpMode
{
    // Declare OpMode members.
    private DcMotorEx carousel = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

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
        if (gamepad1.right_bumper == true) {
            telemetry.addData("Carousel ", "Activated");
            carousel.setPower(0.07);
        }

        if (gamepad1.right_bumper == false) {
            carousel.setPower(0);
        }

        if (gamepad1.left_bumper == true) {
            telemetry.addData("Carousel, ", "Activated");
            carousel.setPower(-0.07);
        }

        if (gamepad1.left_bumper == false) {
            carousel.setPower(0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
