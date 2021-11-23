
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="EncoderTest")
//@Disabled
public class EncoderTest extends LinearOpMode
{
    public DcMotorEx right_front;
    public DcMotorEx left_front;
    public DcMotorEx right_back;
    public DcMotorEx left_back;

    @Override
    public void runOpMode() throws InterruptedException
    {
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");

        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.FORWARD);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);


        // reset encoder counts kept by motors.
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        right_front.setTargetPosition(50);
        left_front.setTargetPosition(50);

        // set motors to run to target encoder position and stop with brakes on.
        right_front.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        right_front.setPower(0.25);
        left_front.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && left_front.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", left_front.getCurrentPosition() + "  busy=" + left_front.isBusy());
            telemetry.addData("encoder-fwd-right", right_front.getCurrentPosition() + "  busy=" + right_front.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        left_front.setPower(0.0);
        right_front.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", right_front.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", left_front.getCurrentPosition());
            telemetry.update();
            idle();
        }


    }
}
